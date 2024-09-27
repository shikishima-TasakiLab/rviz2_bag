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
      &RViz2Bag_Player::pbtn__rosbag_open__callback
    );
    connect(
      ui_player_->spin__rosbag_starttime,
      static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
      ui_player_->hsld__rosbag_starttime,
      &QSlider::setValue
    );
    connect(
      ui_player_->hsld__rosbag_starttime,
      &QSlider::valueChanged,
      ui_player_->spin__rosbag_starttime,
      &QSpinBox::setValue
    );
    connect(
      ui_player_->pbtn__select_all,
      &QPushButton::clicked,
      this,
      &RViz2Bag_Player::pbtn__select_all__callback
    );
    connect(
      ui_player_->pbtn__deselect_all,
      &QPushButton::clicked,
      this,
      &RViz2Bag_Player::pbtn__deselect_all__callback
    );
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

    storage_options = std::make_shared<rosbag2_storage::StorageOptions>();
    storage_options->uri = dir.toLocal8Bit().constData();
    storage_options->storage_id = "";
    storage_options->storage_config_uri = "";

    // Find metadata
    rosbag2_storage::MetadataIo metadata_io;
    if (metadata_io.metadata_file_exists(storage_options->uri) == false)
    {
      ui_player_->ledit__rosbag_dir->setText(QString("Error: metadata not found."));
      return;
    }

    auto metadata = metadata_io.read_metadata(storage_options->uri);

    // Set starttime
    int starttime_max = metadata.duration.count() / 1000000000;
    ui_player_->hsld__rosbag_starttime->setValue(0);
    ui_player_->spin__rosbag_starttime->setValue(0);
    ui_player_->hsld__rosbag_starttime->setMaximum(starttime_max);
    ui_player_->spin__rosbag_starttime->setMaximum(starttime_max);

    // Reset Topic List
    int list_count = ui_player_->list__rosbag_topic->count();
    for (int i = 0; i < list_count; i++) {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->takeItem(0);
      delete item;
    }

    // Load Topic List
    for (rosbag2_storage::TopicInformation topic_with_message_count : metadata.topics_with_message_count) {
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
    for (int i = 0; i < list_count; i++) {
      QListWidgetItem *item = ui_player_->list__rosbag_topic->item(i);
      item->setCheckState(state);
    }
  }

} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Player, rviz_common::Panel)
