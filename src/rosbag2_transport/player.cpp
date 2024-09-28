// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Modifications copyright (C) 2024 <shikishima-TasakiLab>

#include "rviz2_bag/rosbag2_transport/player.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rcl/graph.h>

#include <rclcpp/rclcpp.hpp>

#include <rcutils/time.h>

#include <rosbag2_cpp/clocks/time_controller_clock.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

#include <rosbag2_storage/storage_filter.hpp>

#include <rosbag2_transport/qos.hpp>

namespace
{
/**
 * Trivial std::unique_lock wrapper providing constructor that allows Clang Thread Safety Analysis.
 * The std::unique_lock does not have these annotations.
 */
class RCPPUTILS_TSA_SCOPED_CAPABILITY TSAUniqueLock : public std::unique_lock<std::mutex>
{
public:
  explicit TSAUniqueLock(std::mutex & mu) RCPPUTILS_TSA_ACQUIRE(mu)
  : std::unique_lock<std::mutex>(mu)
  {}

  ~TSAUniqueLock() RCPPUTILS_TSA_RELEASE() {}
};

/**
 * Determine which QoS to offer for a topic.
 * The priority of the profile selected is:
 *   1. The override specified in play_options (if one exists for the topic).
 *   2. A profile automatically adapted to the recorded QoS profiles of publishers on the topic.
 *
 * \param topic_name The full name of the topic, with namespace (ex. /arm/joint_status).
 * \param topic_qos_profile_overrides A map of topic to QoS profile overrides.
 * @return The QoS profile to be used for subscribing.
 */
rclcpp::QoS publisher_qos_for_topic(
  const rosbag2_storage::TopicMetadata & topic,
  const std::unordered_map<std::string, rclcpp::QoS> & topic_qos_profile_overrides,
  const rclcpp::Logger & logger)
{
  using rosbag2_transport::Rosbag2QoS;
  auto qos_it = topic_qos_profile_overrides.find(topic.name);
  if (qos_it != topic_qos_profile_overrides.end()) {
    RCLCPP_INFO_STREAM(
      logger,
      "[rviz2_bag] Overriding QoS profile for topic " << topic.name);
    return Rosbag2QoS{qos_it->second};
  } else if (topic.offered_qos_profiles.empty()) {
    return Rosbag2QoS{};
  }

  const auto profiles_yaml = YAML::Load(topic.offered_qos_profiles);
  const auto offered_qos_profiles = profiles_yaml.as<std::vector<Rosbag2QoS>>();
  return Rosbag2QoS::adapt_offer_to_recorded_offers(topic.name, offered_qos_profiles);
}
}  // namespace

namespace rosbag2_transport
{

Player::Player(
  std::unique_ptr<rosbag2_cpp::Reader> reader,
  const rosbag2_storage::StorageOptions & storage_options,
  const rosbag2_transport::PlayOptions & play_options,
  const rclcpp::Node::SharedPtr node_handle)
: storage_options_(storage_options),
  play_options_(play_options)
{
  nh_ = node_handle;

  {
    std::lock_guard<std::mutex> lk(reader_mutex_);
    reader_ = std::move(reader);
    // keep reader open until player is destroyed
    reader_->open(storage_options_, {"", rmw_get_serialization_format()});
    auto metadata = reader_->get_metadata();
    starting_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      metadata.starting_time.time_since_epoch()).count();
    // If a non-default (positive) starting time offset is provided in PlayOptions,
    // then add the offset to the starting time obtained from reader metadata
    if (play_options_.start_offset < 0) {
      RCLCPP_WARN_STREAM(
        nh_->get_logger(),
        "[rviz2_bag] Invalid start offset value: " <<
          RCUTILS_NS_TO_S(static_cast<double>(play_options_.start_offset)) <<
          ". Negative start offset ignored.");
    } else {
      starting_time_ += play_options_.start_offset;
    }
    clock_ = std::make_unique<rosbag2_cpp::TimeControllerClock>(
      starting_time_, std::chrono::steady_clock::now,
      std::chrono::milliseconds{100}, play_options_.start_paused);
    set_rate(play_options_.rate);
    topic_qos_profile_overrides_ = play_options_.topic_qos_profile_overrides;
    prepare_publishers();
  }
}

Player::~Player()
{
  // closes reader
  std::lock_guard<std::mutex> lk(reader_mutex_);
  if (reader_) {
    reader_->close();
  }
}

const std::chrono::milliseconds
Player::queue_read_wait_period_ = std::chrono::milliseconds(100);

bool Player::is_storage_completely_loaded() const
{
  if (storage_loading_future_.valid() &&
    storage_loading_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    storage_loading_future_.get();
  }
  return !storage_loading_future_.valid();
}

void Player::play()
{
  rclcpp::Duration delay(0, 0);
  if (play_options_.delay >= rclcpp::Duration(0, 0)) {
    delay = play_options_.delay;
  } else {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(),
      "[rviz2_bag] Invalid delay value: " << play_options_.delay.nanoseconds() << ". Delay is disabled.");
  }

  try {
    do {
      if (delay > rclcpp::Duration(0, 0)) {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Sleep " << delay.nanoseconds() << " ns");
        std::chrono::nanoseconds duration(delay.nanoseconds());
        std::this_thread::sleep_for(duration);
      }
      {
        std::lock_guard<std::mutex> lk(reader_mutex_);
        reader_->seek(starting_time_);
        clock_->jump(starting_time_);
      }
      storage_loading_future_ = std::async(std::launch::async, [this]() {load_storage_content();});
      wait_for_filled_queue();
      play_messages_from_queue();
      {
        std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
        is_ready_to_play_from_queue_ = false;
        ready_to_play_from_queue_cv_.notify_all();
      }
    } while (rclcpp::ok() && no_stop_request_ && play_options_.loop);
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR(nh_->get_logger(), "[rviz2_bag] Failed to play: %s", e.what());
  }
  std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
  is_ready_to_play_from_queue_ = false;
  ready_to_play_from_queue_cv_.notify_all();

  // Wait for all published messages to be acknowledged.
  if (play_options_.wait_acked_timeout >= 0) {
    std::chrono::milliseconds timeout(play_options_.wait_acked_timeout);
    if (timeout == std::chrono::milliseconds(0)) {
      timeout = std::chrono::milliseconds(-1);
    }
    for (auto pub : publishers_) {
      try {
        if (!pub.second->generic_publisher()->wait_for_all_acked(timeout)) {
          RCLCPP_ERROR(
            nh_->get_logger(),
            "[rviz2_bag] "
            "Timed out while waiting for all published messages to be acknowledged for topic %s",
            pub.first.c_str());
        }
      } catch (std::exception & e) {
        RCLCPP_ERROR(
          nh_->get_logger(),
          "[rviz2_bag] "
          "Exception occurred while waiting for all published messages to be acknowledged for "
          "topic %s : %s",
          pub.first.c_str(),
          e.what());
      }
    }
  }

  stopped();
}

void Player::pause()
{
  clock_updated_enable_ = false;
  clock_->pause();
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Pause");
}

void Player::stop()
{
  clock_updated_enable_ = false;
  no_stop_request_ = false;
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Stop");
}

void Player::resume()
{
  clock_updated_enable_ = true;
  clock_->resume();
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Resume");
}

void Player::toggle_paused()
{
  is_paused() ? resume() : pause();
}

bool Player::is_paused() const
{
  return clock_->is_paused();
}

double Player::get_rate() const
{
  return clock_->get_rate();
}

bool Player::set_rate(double rate)
{
  bool ok = clock_->set_rate(rate);
  if (ok) {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Set rate to " << rate);
  } else {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[rviz2_bag] Failed to set rate to invalid value " << rate);
  }
  return ok;
}

rosbag2_storage::SerializedBagMessageSharedPtr Player::peek_next_message_from_queue()
{
  rosbag2_storage::SerializedBagMessageSharedPtr * message_ptr_ptr = message_queue_.peek();
  while (message_ptr_ptr == nullptr && !is_storage_completely_loaded() && rclcpp::ok() && no_stop_request_) {
    RCLCPP_WARN_THROTTLE(
      nh_->get_logger(),
      *nh_->get_clock(),
      1000,
      "[rviz2_bag] Message queue starved. Messages will be delayed. Consider "
      "increasing the --read-ahead-queue-size option.");

    std::this_thread::sleep_for(std::chrono::microseconds(100));
    message_ptr_ptr = message_queue_.peek();
  }

  // Workaround for race condition between peek and is_storage_completely_loaded()
  // Don't sync with mutex for the sake of the performance
  if (message_ptr_ptr == nullptr) {
    message_ptr_ptr = message_queue_.peek();
  }

  if (message_ptr_ptr != nullptr) {
    return *message_ptr_ptr;
  }
  return nullptr;
}

bool Player::play_next()
{
  if (!clock_->is_paused()) {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[rviz2_bag] Called play next, but not in paused state.");
    return false;
  }

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] Playing next message.");

  // Temporary take over playback from play_messages_from_queue()
  std::lock_guard<std::mutex> main_play_loop_lk(skip_message_in_main_play_loop_mutex_);
  skip_message_in_main_play_loop_ = true;
  // Wait for player to be ready for playback messages from queue i.e. wait for Player:play() to
  // be called if not yet and queue to be filled with messages.
  {
    std::unique_lock<std::mutex> lk(ready_to_play_from_queue_mutex_);
    ready_to_play_from_queue_cv_.wait(lk, [this] {return is_ready_to_play_from_queue_;});
  }

  rosbag2_storage::SerializedBagMessageSharedPtr message_ptr = peek_next_message_from_queue();

  bool next_message_published = false;
  while (message_ptr != nullptr && !next_message_published) {
    {
      next_message_published = publish_message(message_ptr);
      clock_->jump(message_ptr->time_stamp);
    }
    message_queue_.pop();
    message_ptr = peek_next_message_from_queue();
  }
  clockUpdated(clock_->now());
  return next_message_published;
}

size_t Player::burst(const size_t num_messages)
{
  uint64_t messages_played = 0;

  for (auto ii = 0u; ii < num_messages; ++ii) {
    if (play_next()) {
      ++messages_played;
    } else {
      break;
    }
  }

  return messages_played;
}

void Player::seek(rcutils_time_point_value_t time_point)
{
  // Temporary stop playback in play_messages_from_queue() and block play_next()
  std::lock_guard<std::mutex> main_play_loop_lk(skip_message_in_main_play_loop_mutex_);
  skip_message_in_main_play_loop_ = true;
  // Wait for player to be ready for playback messages from queue i.e. wait for Player:play() to
  // be called if not yet and queue to be filled with messages.
  {
    std::unique_lock<std::mutex> lk(ready_to_play_from_queue_mutex_);
    ready_to_play_from_queue_cv_.wait(lk, [this] {return is_ready_to_play_from_queue_;});
  }
  cancel_wait_for_next_message_ = true;
  // if given seek value is earlier than the beginning of the bag, then clamp
  // it to the beginning of the bag
  if (time_point < starting_time_) {
    time_point = starting_time_;
  }
  {
    std::lock_guard<std::mutex> lk(reader_mutex_);
    // Purge current messages in queue.
    while (message_queue_.pop()) {}
    reader_->seek(time_point);
    clock_->jump(time_point);
    // Restart queuing thread if it has finished running (previously reached end of bag),
    // otherwise, queueing should continue automatically after releasing mutex
    if (is_storage_completely_loaded() && rclcpp::ok() && no_stop_request_) {
      storage_loading_future_ =
        std::async(std::launch::async, [this]() {load_storage_content();});
    }
  }
}

void Player::wait_for_filled_queue() const
{
  while (
    message_queue_.size_approx() < play_options_.read_ahead_queue_size &&
    !is_storage_completely_loaded() && rclcpp::ok() && no_stop_request_)
  {
    std::this_thread::sleep_for(queue_read_wait_period_);
  }
}

void Player::load_storage_content()
{
  auto queue_lower_boundary =
    static_cast<size_t>(play_options_.read_ahead_queue_size * read_ahead_lower_bound_percentage_);
  auto queue_upper_boundary = play_options_.read_ahead_queue_size;

  while (rclcpp::ok() && no_stop_request_) {
    TSAUniqueLock lk(reader_mutex_);
    if (!reader_->has_next()) {break;}

    if (message_queue_.size_approx() < queue_lower_boundary) {
      enqueue_up_to_boundary(queue_upper_boundary);
    } else {
      lk.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Player::enqueue_up_to_boundary(size_t boundary)
{
  rosbag2_storage::SerializedBagMessageSharedPtr message;
  for (size_t i = message_queue_.size_approx(); i < boundary; i++) {
    if (!reader_->has_next()) {
      break;
    }
    message = reader_->read_next();
    message_queue_.enqueue(message);
  }
}

void Player::play_messages_from_queue()
{
  // Note: We need to use message_queue_.peek() instead of message_queue_.try_dequeue(message)
  // to support play_next() API logic.
  rosbag2_storage::SerializedBagMessageSharedPtr message_ptr = peek_next_message_from_queue();
  { // Notify play_next() that we are ready for playback
    // Note: We should do notification that we are ready for playback after peeking pointer to
    // the next message. message_queue_.peek() is not allowed to be called from more than one
    // thread concurrently.
    std::lock_guard<std::mutex> lk(ready_to_play_from_queue_mutex_);
    is_ready_to_play_from_queue_ = true;
    ready_to_play_from_queue_cv_.notify_all();
  }
  while (message_ptr != nullptr && rclcpp::ok() && no_stop_request_) {
    // Do not move on until sleep_until returns true
    // It will always sleep, so this is not a tight busy loop on pause
    while (rclcpp::ok() && no_stop_request_ && !clock_->sleep_until(message_ptr->time_stamp)) {
      if (std::atomic_exchange(&cancel_wait_for_next_message_, false)) {
        break;
      }
    }
    std::lock_guard<std::mutex> lk(skip_message_in_main_play_loop_mutex_);
    if (rclcpp::ok() && no_stop_request_) {
      if (skip_message_in_main_play_loop_) {
        skip_message_in_main_play_loop_ = false;
        cancel_wait_for_next_message_ = false;
        message_ptr = peek_next_message_from_queue();
        continue;
      }
      publish_message(message_ptr);
    }
    message_queue_.pop();
    message_ptr = peek_next_message_from_queue();
  }
  // while we're in pause state, make sure we don't return
  // if we happen to be at the end of queue
  while (is_paused() && rclcpp::ok() && no_stop_request_) {
    clock_->sleep_until(clock_->now());
  }
}

void Player::prepare_publishers()
{
  rosbag2_storage::StorageFilter storage_filter;
  storage_filter.topics = play_options_.topics_to_filter;
  reader_->set_filter(storage_filter);

  // Create /clock publisher
  if (play_options_.clock_publish_frequency > 0.f) {
    const auto publish_period = std::chrono::nanoseconds(
      static_cast<uint64_t>(RCUTILS_S_TO_NS(1) / play_options_.clock_publish_frequency));
    // NOTE: PlayerClock does not own this publisher because rosbag2_cpp
    // should not own transport-based functionality
    clock_publisher_ = nh_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());
    clock_publish_timer_ = nh_->create_wall_timer(
      publish_period, [this]() {
        auto msg = rosgraph_msgs::msg::Clock();
        msg.clock = rclcpp::Time(clock_->now());
        clock_publisher_->publish(msg);
      });
  }

  clock_update_timer_ = nh_->create_wall_timer(
    std::chrono::milliseconds(100),
    [this](){if (clock_updated_enable_) clockUpdated(clock_->now());}
  );

  // Create topic publishers
  auto topics = reader_->get_all_topics_and_types();
  std::string topic_without_support_acked;
  for (const auto & topic : topics) {
    if (publishers_.find(topic.name) != publishers_.end()) {
      continue;
    }
    // filter topics to add publishers if necessary
    auto & filter_topics = storage_filter.topics;
    if (!filter_topics.empty()) {
      auto iter = std::find(filter_topics.begin(), filter_topics.end(), topic.name);
      if (iter == filter_topics.end()) {
        continue;
      }
    }

    auto topic_qos = publisher_qos_for_topic(
      topic, topic_qos_profile_overrides_,
      nh_->get_logger());
    try {
      std::shared_ptr<rclcpp::GenericPublisher> pub =
        nh_->create_generic_publisher(topic.name, topic.type, topic_qos);
      std::shared_ptr<Player::PlayerPublisher> player_pub =
        std::make_shared<Player::PlayerPublisher>(
        std::move(pub), play_options_.disable_loan_message);
      publishers_.insert(std::make_pair(topic.name, player_pub));
      if (play_options_.wait_acked_timeout >= 0 &&
        topic_qos.reliability() == rclcpp::ReliabilityPolicy::BestEffort)
      {
        topic_without_support_acked += topic.name + ", ";
      }
    } catch (const std::runtime_error & e) {
      // using a warning log seems better than adding a new option
      // to ignore some unknown message type library
      RCLCPP_WARN(
        nh_->get_logger(),
        "[rviz2_bag] Ignoring a topic '%s', reason: %s.", topic.name.c_str(), e.what());
    }
  }

  if (!topic_without_support_acked.empty()) {
    // remove the last ", "
    topic_without_support_acked.erase(
      topic_without_support_acked.end() - 2,
      topic_without_support_acked.end());

    RCLCPP_WARN(
      nh_->get_logger(),
      "[rviz2_bag] --wait-for-all-acked is invalid for the below topics since reliability of QOS is "
      "BestEffort.\n%s", topic_without_support_acked.c_str());
  }

  // Create a publisher and callback for when encountering a split in the input
  split_event_pub_ = nh_->create_publisher<rosbag2_interfaces::msg::ReadSplitEvent>(
    "events/read_split",
    1);
  rosbag2_cpp::bag_events::ReaderEventCallbacks callbacks;
  callbacks.read_split_callback =
    [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
      auto message = rosbag2_interfaces::msg::ReadSplitEvent();
      message.closed_file = info.closed_file;
      message.opened_file = info.opened_file;
      split_event_pub_->publish(message);
    };
  reader_->add_event_callbacks(callbacks);
}

bool Player::publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message)
{
  bool message_published = false;
  auto publisher_iter = publishers_.find(message->topic_name);
  if (publisher_iter != publishers_.end()) {
    try {
      publisher_iter->second->publish(rclcpp::SerializedMessage(*message->serialized_data));
      message_published = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(), "[rviz2_bag] Failed to publish message on '" << message->topic_name <<
          "' topic. \nError: %s" << e.what());
    }
  }
  return message_published;
}

}  // namespace rosbag2_transport
