#include "rviz2_bag/player.hpp"

namespace rviz2_bag
{

  RViz2Bag_Player::RViz2Bag_Player(QWidget *parent)
      : rviz_common::Panel(parent), ui_player_(new Ui::Player())
  {
    ui_player_->setupUi(this);

    connect(
        ui_player_->pbtn__rosbag_open,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__rosbag_open__callback);
    connect(
        ui_player_->dspin__rosbag_rate,
        static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        this,
        &RViz2Bag_Player::dspin__rosbag_rate__valueChanged);
    connect(
        ui_player_->dspin__rosbag_elapsed_time,
        static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        this,
        &RViz2Bag_Player::dspin__rosbag_elapsed_time__valueChanged);
    connect(
        ui_player_->dspin__rosbag_elapsed_time,
        &QDoubleSpinBox::editingFinished,
        this,
        &RViz2Bag_Player::dspin__rosbag_elapsed_time__editingFinished);
    connect(
        ui_player_->hsld__rosbag_elapsed_time,
        &QSlider::valueChanged,
        this,
        &RViz2Bag_Player::hsld__rosbag_elapsed_time__valueChanged);
    connect(
        ui_player_->hsld__rosbag_elapsed_time,
        &QSlider::sliderReleased,
        this,
        &RViz2Bag_Player::hsld__rosbag_elapsed_time__sliderReleased);
    connect(
        ui_player_->pbtn__select_all,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__select_all__callback);
    connect(
        ui_player_->pbtn__deselect_all,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__deselect_all__callback);
    connect(
        ui_player_->pbtn__rosbag_play,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__rosbag_play__callback);
    connect(
        ui_player_->pbtn__rosbag_stop,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__rosbag_stop__callback);
    connect(
        ui_player_->pbtn__rosbag_pause,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__rosbag_pause__callback);
    connect(
        ui_player_->pbtn__backward,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__backward__callback);
    connect(
        ui_player_->pbtn__play_next,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__play_next__callback);
  }

  RViz2Bag_Player::~RViz2Bag_Player()
  {
    stop();
  }

  void RViz2Bag_Player::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  void RViz2Bag_Player::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void RViz2Bag_Player::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
  }

  void RViz2Bag_Player::dspin__rosbag_rate__valueChanged(double value)
  {
    if (bag_player_ == nullptr)
    {
    }
    else
    {
      bag_player_->set_rate(value);
    }
  }

  void RViz2Bag_Player::dspin__rosbag_elapsed_time__valueChanged(double value)
  {
    if (bag_player_ == nullptr)
    {
      ui_player_->hsld__rosbag_elapsed_time->setValue(value * 100); //  [s]->[10ms]
    }
    else if (bag_player_->is_paused())
    {
      ui_player_->hsld__rosbag_elapsed_time->setValue(value * 100); //  [s]->[10ms]
    }
    else
    {
    }
  }

  void RViz2Bag_Player::dspin__rosbag_elapsed_time__editingFinished()
  {
    if (bag_player_ == nullptr)
    {
    }
    else if (bag_player_->is_paused())
    {
    }
    else
    {
      auto elapsed_time = static_cast<rcutils_time_point_value_t>(ui_player_->dspin__rosbag_elapsed_time->value() * 1e9); // [s]->[ns]
      bag_player_->seek(elapsed_time + metadata_->starting_time.time_since_epoch().count());
    }
  }

  void RViz2Bag_Player::hsld__rosbag_elapsed_time__valueChanged(int value)
  {
    if (bag_player_ == nullptr)
    {
      ui_player_->dspin__rosbag_elapsed_time->setValue(value / 100.0); //  [10ms]->[s]
    }
    else if (bag_player_->is_paused())
    {
      ui_player_->dspin__rosbag_elapsed_time->setValue(value / 100.0); //  [10ms]->[s]
    }
    else
    {
    }
  }

  void RViz2Bag_Player::hsld__rosbag_elapsed_time__sliderReleased()
  {
    if (bag_player_ == nullptr)
    {
    }
    else if (bag_player_->is_paused())
    {
    }
    else
    {
      auto elapsed_time = static_cast<rcutils_time_point_value_t>(ui_player_->hsld__rosbag_elapsed_time->value()) * 10000000; // [10ms]->[ns]
      bag_player_->seek(elapsed_time + metadata_->starting_time.time_since_epoch().count());
    }
  }

  void RViz2Bag_Player::pbtn__rosbag_open__callback()
  {
    // Open ROSBAG dir
    QString dir = QFileDialog::getExistingDirectory(this, tr("Open ROSBAG"), "~", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    ui_player_->ledit__rosbag_dir->setText(dir);

    storage_options_ = std::make_unique<rosbag2_storage::StorageOptions>();
    storage_options_->uri = dir.toLocal8Bit().constData();
    storage_options_->storage_id = "";
    storage_options_->storage_config_uri = "";

    // Find metadata
    rosbag2_storage::MetadataIo metadata_io;
    if (metadata_io.metadata_file_exists(storage_options_->uri) == false)
    {
      ui_player_->ledit__rosbag_dir->setText(QString("Error: metadata not found."));
      return;
    }

    metadata_ = std::make_unique<rosbag2_storage::BagMetadata>(metadata_io.read_metadata(storage_options_->uri));

    // Set elapsed time
    int elapsed_time_max = metadata_->duration.count() / 10000000;
    ui_player_->hsld__rosbag_elapsed_time->setValue(0);
    ui_player_->dspin__rosbag_elapsed_time->setValue(0);
    ui_player_->hsld__rosbag_elapsed_time->setMaximum(elapsed_time_max);
    ui_player_->dspin__rosbag_elapsed_time->setMaximum(elapsed_time_max / 100.0);

    // Reset Topic List
    int list_count = ui_player_->list__rosbag_topic->count();
    for (int i = 0; i < list_count; i++)
    {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->takeItem(0);
      delete item;
    }

    // Load Topic List
    for (rosbag2_storage::TopicInformation topic_with_message_count : metadata_->topics_with_message_count)
    {
      QListWidgetItem *listWidgetItem = new QListWidgetItem();
      listWidgetItem->setFlags(listWidgetItem->flags() | Qt::ItemIsUserCheckable);
      listWidgetItem->setCheckState(Qt::Checked);
      listWidgetItem->setText(topic_with_message_count.topic_metadata.name.c_str());
      ui_player_->list__rosbag_topic->addItem(listWidgetItem);
    }

    // Enable UI
    ui_player_->hsld__rosbag_elapsed_time->setEnabled(true);
    ui_player_->dspin__rosbag_elapsed_time->setEnabled(true);
    ui_player_->list__rosbag_topic->setEnabled(true);

    ui_player_->pbtn__rosbag_open->setEnabled(true);
    ui_player_->dspin__rosbag_clock->setEnabled(true);
    ui_player_->dspin__rosbag_rate->setEnabled(true);
    ui_player_->check__rosbag_loop->setEnabled(true);
    ui_player_->pbtn__rosbag_play->setEnabled(true);
    ui_player_->pbtn__rosbag_pause->setEnabled(false);
    ui_player_->pbtn__rosbag_stop->setEnabled(false);
    ui_player_->pbtn__backward->setEnabled(true);
    ui_player_->pbtn__play_next->setEnabled(false);
    ui_player_->pbtn__select_all->setEnabled(true);
    ui_player_->pbtn__deselect_all->setEnabled(true);
  }

  void RViz2Bag_Player::pbtn__select_all__callback()
  {
    list_check_all(Qt::Checked);
  }

  void RViz2Bag_Player::pbtn__deselect_all__callback()
  {
    list_check_all(Qt::Unchecked);
  }

  void RViz2Bag_Player::list_check_all(Qt::CheckState state)
  {
    int list_count = ui_player_->list__rosbag_topic->count();
    for (int i = 0; i < list_count; i++)
    {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->item(i);
      item->setCheckState(state);
    }
  }

  bool RViz2Bag_Player::play()
  {
    rosbag2_transport::PlayOptions play_options;

    play_options.read_ahead_queue_size = 1000;
    play_options.node_prefix = "";
    play_options.rate = ui_player_->dspin__rosbag_rate->value();

    play_options.topics_to_filter = {};
    int list_count = ui_player_->list__rosbag_topic->count();
    for (int i = 0; i < list_count; i++)
    {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->item(i);
      if (item->checkState() == Qt::Checked)
      {
        play_options.topics_to_filter.push_back(item->text().toStdString());
      }
    }
    if (play_options.topics_to_filter.size() < 1)
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "[rviz2_bag] No topic selected.");
      return false;
    }

    play_options.topic_qos_profile_overrides = {};
    play_options.loop = (ui_player_->check__rosbag_loop->checkState() == Qt::Checked);
    play_options.topic_remapping_options = {};
    play_options.clock_publish_frequency = ui_player_->dspin__rosbag_clock->value();
    play_options.delay = rclcpp::Duration(0, 0);
    play_options.start_paused = false;
    play_options.start_offset = static_cast<rcutils_time_point_value_t>(ui_player_->hsld__rosbag_elapsed_time->value()) * 10000000; // [10ms]->[ns]
    play_options.disable_keyboard_controls = true;
    play_options.wait_acked_timeout = -1;
    play_options.disable_loan_message = false;

    auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(*storage_options_);
    bag_player_ = std::make_shared<rosbag2_transport::Player>(
        std::move(reader),
        *storage_options_,
        play_options,
        nh_);

    connect(
        bag_player_.get(),
        &rosbag2_transport::Player::stopped,
        this,
        &RViz2Bag_Player::pbtn__rosbag_stop__callback);

    connect(
        bag_player_.get(),
        &rosbag2_transport::Player::clockUpdated,
        this,
        &RViz2Bag_Player::update_elapsed_time__callback);

    spin_thread_ = std::make_unique<std::thread>([this]()
                                                 { this->bag_player_->play(); });

    return true;
  }

  void RViz2Bag_Player::stop()
  {
    if (bag_player_ == nullptr)
    {
    }
    else
    {
      bag_player_->stop();
      spin_thread_->join();

      bag_player_.reset();
      spin_thread_.reset();
    }
  }

  void RViz2Bag_Player::pbtn__rosbag_play__callback()
  {
    if (bag_player_ == nullptr)
    {
      if (play() == false)
        return;
    }
    else
    {
      bool need_to_seek = (hsld__rosbag_elapsed_time__value__pause_ != ui_player_->hsld__rosbag_elapsed_time->value());
      bag_player_->resume();
      if (need_to_seek)
        hsld__rosbag_elapsed_time__sliderReleased();
    }

    ui_player_->pbtn__rosbag_open->setEnabled(false);
    ui_player_->dspin__rosbag_clock->setEnabled(false);
    ui_player_->dspin__rosbag_rate->setEnabled(true);
    ui_player_->check__rosbag_loop->setEnabled(false);
    ui_player_->pbtn__rosbag_play->setEnabled(false);
    ui_player_->pbtn__rosbag_pause->setEnabled(true);
    ui_player_->pbtn__rosbag_stop->setEnabled(true);
    ui_player_->pbtn__backward->setEnabled(true);
    ui_player_->pbtn__play_next->setEnabled(false);
    ui_player_->pbtn__select_all->setEnabled(false);
    ui_player_->pbtn__deselect_all->setEnabled(false);
  }

  void RViz2Bag_Player::pbtn__rosbag_stop__callback()
  {
    stop();

    ui_player_->dspin__rosbag_elapsed_time->setValue(0.0);
    ui_player_->hsld__rosbag_elapsed_time->setValue(0);

    ui_player_->pbtn__rosbag_open->setEnabled(true);
    ui_player_->dspin__rosbag_clock->setEnabled(true);
    ui_player_->dspin__rosbag_rate->setEnabled(true);
    ui_player_->check__rosbag_loop->setEnabled(true);
    ui_player_->pbtn__rosbag_play->setEnabled(true);
    ui_player_->pbtn__rosbag_pause->setEnabled(false);
    ui_player_->pbtn__rosbag_stop->setEnabled(false);
    ui_player_->pbtn__backward->setEnabled(true);
    ui_player_->pbtn__play_next->setEnabled(false);
    ui_player_->pbtn__select_all->setEnabled(true);
    ui_player_->pbtn__deselect_all->setEnabled(true);
  }

  void RViz2Bag_Player::pbtn__rosbag_pause__callback()
  {
    if (bag_player_ == nullptr)
    {
    }
    else
    {
      bag_player_->pause();

      ui_player_->pbtn__rosbag_open->setEnabled(false);
      ui_player_->dspin__rosbag_clock->setEnabled(false);
      ui_player_->dspin__rosbag_rate->setEnabled(true);
      ui_player_->check__rosbag_loop->setEnabled(false);
      ui_player_->pbtn__rosbag_play->setEnabled(true);
      ui_player_->pbtn__rosbag_pause->setEnabled(false);
      ui_player_->pbtn__rosbag_stop->setEnabled(true);
      ui_player_->pbtn__backward->setEnabled(true);
      ui_player_->pbtn__play_next->setEnabled(true);
      ui_player_->pbtn__select_all->setEnabled(false);
      ui_player_->pbtn__deselect_all->setEnabled(false);
    }
  }

  void RViz2Bag_Player::pbtn__backward__callback()
  {
    if (bag_player_ == nullptr)
    {
      ui_player_->dspin__rosbag_elapsed_time->setValue(0.0);
      ui_player_->hsld__rosbag_elapsed_time->setValue(0);
    }
    else if (bag_player_->is_paused())
    {
      ui_player_->dspin__rosbag_elapsed_time->setValue(0.0);
      ui_player_->hsld__rosbag_elapsed_time->setValue(0);
    }
    else
    {
      bag_player_->seek(metadata_->starting_time.time_since_epoch().count());
    }
  }

  void RViz2Bag_Player::pbtn__play_next__callback()
  {
    if (bag_player_ == nullptr)
    {
    }
    else if (bag_player_->is_paused())
    {
      bag_player_->play_next();
    }
    else
    {
    }
  }

  void RViz2Bag_Player::update_elapsed_time__callback(rcutils_time_point_value_t time_point)
  {
    auto elapsed_time_ns = std::chrono::nanoseconds(time_point) - metadata_->starting_time.time_since_epoch();
    auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time_ns);

    hsld__rosbag_elapsed_time__value__pause_ = elapsed_time_ms.count() / 10; // [10ms]
    ui_player_->hsld__rosbag_elapsed_time->setValue(hsld__rosbag_elapsed_time__value__pause_);
    if (ui_player_->dspin__rosbag_elapsed_time->hasFocus() == false)
    {
      ui_player_->dspin__rosbag_elapsed_time->setValue(elapsed_time_ms.count() / 1000.0);
    }
  }

} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Player, rviz_common::Panel)
