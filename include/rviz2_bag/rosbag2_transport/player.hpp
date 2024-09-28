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

#ifndef RVIZ2_BAG__ROSBAG2_TRANSPORT__PLAYER_HPP_
#define RVIZ2_BAG__ROSBAG2_TRANSPORT__PLAYER_HPP_

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <moodycamel/readerwriterqueue.h>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>

#include <rosbag2_cpp/clocks/player_clock.hpp>
#include <rosbag2_interfaces/msg/read_split_event.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/visibility_control.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <QtCore/QObject>


namespace rosbag2_cpp
{
class Reader;
}  // namespace rosbag2_cpp

namespace rosbag2_transport
{
class Player: public QObject
{
  Q_OBJECT
public:
  ROSBAG2_TRANSPORT_PUBLIC
  Player(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options,
    const rclcpp::Node::SharedPtr node_handle);

  ROSBAG2_TRANSPORT_PUBLIC
  virtual ~Player();

  ROSBAG2_TRANSPORT_PUBLIC
  void play();

  // Playback control interface
  /// Pause the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual void pause();

  ROSBAG2_TRANSPORT_PUBLIC
  virtual void stop();

  /// Start the flow of time for playback.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual void resume();

  /// Pause if time running, resume if paused.
  ROSBAG2_TRANSPORT_PUBLIC
  void toggle_paused();

  /// Return whether the playback is currently paused.
  ROSBAG2_TRANSPORT_PUBLIC
  bool is_paused() const;

  /// Return current playback rate.
  ROSBAG2_TRANSPORT_PUBLIC
  double get_rate() const;

  /// \brief Set the playback rate.
  /// \return false if an invalid value was provided (<= 0).
  ROSBAG2_TRANSPORT_PUBLIC
  virtual bool set_rate(double);

  /// \brief Playing next message from queue when in pause.
  /// \details This is blocking call and it will wait until next available message will be
  /// published or rclcpp context shut down.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return true if player in pause mode and successfully played next message, otherwise false.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual bool play_next();

  /// \brief Burst the next \p num_messages messages from the queue when paused.
  /// \param num_messages The number of messages to burst from the queue.
  /// \details This call will play the next \p num_messages from the queue in burst mode. The
  /// timing of the messages is ignored.
  /// \note If internal player queue is starving and storage has not been completely loaded,
  /// this method will wait until new element will be pushed to the queue.
  /// \return The number of messages that was played.
  ROSBAG2_TRANSPORT_PUBLIC
  virtual size_t burst(const size_t num_messages);

  /// \brief Advance player to the message with closest timestamp >= time_point.
  /// \details This is blocking call and it will wait until current message will be published
  /// and message queue will be refilled.
  /// If time_point is before the beginning of the bag, then playback time will be set to the
  /// beginning of the bag.
  /// If time_point is after the end of the bag, playback time will be set to the end of the bag,
  /// which will then end playback, or if loop is enabled then will start playing at the beginning
  /// of the next loop.
  /// \param time_point Time point in ROS playback timeline.
  ROSBAG2_TRANSPORT_PUBLIC
  void seek(rcutils_time_point_value_t time_point);

Q_SIGNALS:
  void clockUpdated(rcutils_time_point_value_t time_point);
  void stopped();

protected:
  class PlayerPublisher final
  {
public:
    explicit PlayerPublisher(
      std::shared_ptr<rclcpp::GenericPublisher> pub,
      bool disable_loan_message)
    : publisher_(std::move(pub))
    {
      using std::placeholders::_1;
      if (disable_loan_message || !publisher_->can_loan_messages()) {
        publish_func_ = std::bind(&rclcpp::GenericPublisher::publish, publisher_, _1);
      } else {
        publish_func_ = std::bind(&rclcpp::GenericPublisher::publish_as_loaned_msg, publisher_, _1);
      }
    }

    ~PlayerPublisher() {}

    void publish(const rclcpp::SerializedMessage & message)
    {
      publish_func_(message);
    }

    std::shared_ptr<rclcpp::GenericPublisher> generic_publisher()
    {
      return publisher_;
    }
  
private:
    std::shared_ptr<rclcpp::GenericPublisher> publisher_;
    std::function<void(const rclcpp::SerializedMessage &)> publish_func_;
  };

  rclcpp::Node::SharedPtr nh_;

  bool is_ready_to_play_from_queue_{false};
  std::mutex ready_to_play_from_queue_mutex_;
  std::condition_variable ready_to_play_from_queue_cv_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  std::unordered_map<std::string, std::shared_ptr<PlayerPublisher>> publishers_;

private:
  bool no_stop_request{true};
  rosbag2_storage::SerializedBagMessageSharedPtr peek_next_message_from_queue();
  void load_storage_content();
  bool is_storage_completely_loaded() const;
  void enqueue_up_to_boundary(size_t boundary) RCPPUTILS_TSA_REQUIRES(reader_mutex_);
  void wait_for_filled_queue() const;
  void play_messages_from_queue();
  void prepare_publishers();
  bool publish_message(rosbag2_storage::SerializedBagMessageSharedPtr message);
  static constexpr double read_ahead_lower_bound_percentage_ = 0.9;
  static const std::chrono::milliseconds queue_read_wait_period_;
  std::atomic_bool cancel_wait_for_next_message_{false};

  std::mutex reader_mutex_;
  std::unique_ptr<rosbag2_cpp::Reader> reader_ RCPPUTILS_TSA_GUARDED_BY(reader_mutex_);

  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_transport::PlayOptions play_options_;
  moodycamel::ReaderWriterQueue<rosbag2_storage::SerializedBagMessageSharedPtr> message_queue_;
  mutable std::future<void> storage_loading_future_;
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
  std::unique_ptr<rosbag2_cpp::PlayerClock> clock_;
  std::shared_ptr<rclcpp::TimerBase> clock_publish_timer_;
  std::shared_ptr<rclcpp::TimerBase> clock_update_timer_;
  std::mutex skip_message_in_main_play_loop_mutex_;
  bool skip_message_in_main_play_loop_ RCPPUTILS_TSA_GUARDED_BY(
    skip_message_in_main_play_loop_mutex_) = false;
  
  rclcpp::Publisher<rosbag2_interfaces::msg::ReadSplitEvent>::SharedPtr split_event_pub_;

  rcutils_time_point_value_t starting_time_;
};

}  // namespace rosbag2_transport

#endif  // RVIZ2_BAG__ROSBAG2_TRANSPORT__PLAYER_HPP_
