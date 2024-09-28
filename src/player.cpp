#include "rviz2_bag/player.hpp"

namespace rviz2_bag
{

  RViz2Bag_Player::RViz2Bag_Player(QWidget *parent)
      : rviz_common::Panel(parent), ui_player_(new Ui::Player())
  {
    ui_player_->setupUi(this);
    // init_qt_panel();
    connect(
        ui_player_->pbtn__rosbag_open,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Player::pbtn__rosbag_open__callback);
    connect(
        ui_player_->spin__rosbag_starttime,
        static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
        ui_player_->hsld__rosbag_starttime,
        &QSlider::setValue);
    connect(
        ui_player_->hsld__rosbag_starttime,
        &QSlider::valueChanged,
        ui_player_->spin__rosbag_starttime,
        &QSpinBox::setValue);
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

  RViz2Bag_Player::~RViz2Bag_Player() {}

  void RViz2Bag_Player::onInitialize()
  {
    // init_ros_node();
  }

  void RViz2Bag_Player::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void RViz2Bag_Player::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
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

    auto metadata = metadata_io.read_metadata(storage_options_->uri);

    // Set starttime
    int starttime_max = metadata.duration.count() / 1000000000;
    ui_player_->hsld__rosbag_starttime->setValue(0);
    ui_player_->spin__rosbag_starttime->setValue(0);
    ui_player_->hsld__rosbag_starttime->setMaximum(starttime_max);
    ui_player_->spin__rosbag_starttime->setMaximum(starttime_max);

    // Reset Topic List
    int list_count = ui_player_->list__rosbag_topic->count();
    for (int i = 0; i < list_count; i++)
    {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->takeItem(0);
      delete item;
    }

    // Load Topic List
    for (rosbag2_storage::TopicInformation topic_with_message_count : metadata.topics_with_message_count)
    {
      QListWidgetItem *listWidgetItem = new QListWidgetItem();
      listWidgetItem->setFlags(listWidgetItem->flags() | Qt::ItemIsUserCheckable);
      listWidgetItem->setCheckState(Qt::Checked);
      listWidgetItem->setText(topic_with_message_count.topic_metadata.name.c_str());
      ui_player_->list__rosbag_topic->addItem(listWidgetItem);
    }

    // Enable UI
    ui_player_->hsld__rosbag_starttime->setEnabled(true);
    ui_player_->spin__rosbag_starttime->setEnabled(true);
    ui_player_->list__rosbag_topic->setEnabled(true);
    ui_player_->pbtn__rosbag_play->setEnabled(true);
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

  void RViz2Bag_Player::pbtn__rosbag_play__callback()
  {
    if (bag_player_ == nullptr) {
      rosbag2_transport::PlayOptions play_options;

      play_options.read_ahead_queue_size = 1000;
      play_options.node_prefix = "";
      play_options.rate = ui_player_->dspin__rosbag_rate->value();
      play_options.topics_to_filter = {};
      play_options.topic_qos_profile_overrides = {};
      play_options.loop = (ui_player_->check__rosbag_loop->checkState() == Qt::Checked);
      play_options.topic_remapping_options = {};
      play_options.clock_publish_frequency = ui_player_->dspin__rosbag_clock->value();
      play_options.delay = rclcpp::Duration(0, 0);
      play_options.start_paused = false;
      play_options.start_offset = 0;
      play_options.disable_keyboard_controls = true;
      play_options.wait_acked_timeout = -1;
      play_options.disable_loan_message = false;

      auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(*storage_options_);
      bag_player_ = std::make_unique<rosbag2_transport::Player>(std::move(reader), *storage_options_, play_options);

    }
    else {

    }
  }

  void RViz2Bag_Player::pbtn__rosbag_stop__callback()
  {
    if (bag_player_ == nullptr) {

    }
    else {
      
    }
  }

  void RViz2Bag_Player::pbtn__rosbag_pause__callback()
  {
  }

  void RViz2Bag_Player::pbtn__backward__callback()
  {
  }

  void RViz2Bag_Player::pbtn__play_next__callback()
  {
  }

} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Player, rviz_common::Panel)
