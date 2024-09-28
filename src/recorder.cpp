#include "rviz2_bag/recorder.hpp"

namespace rviz2_bag
{

  RViz2Bag_Recorder::RViz2Bag_Recorder(QWidget *parent)
      : rviz_common::Panel(parent), ui_recorder_(new Ui::Recorder())
  {
    ui_recorder_->setupUi(this);

    // Setting - Storage
    {
      std::unordered_set<std::string> writer_choices = rosbag2_cpp::plugins::get_class_plugins<rosbag2_storage::storage_interfaces::ReadWriteInterface>();
      std::string default_storage_id = rosbag2_storage::get_default_storage_id();
      std::string default_writer;
      if (writer_choices.find(default_storage_id) != writer_choices.end())
      {
        default_writer = default_storage_id;
      }
      else
      {
        default_writer = *writer_choices.cbegin();
      }

      tree_setting__storage_ = new QTreeWidgetItem();
      tree_setting__storage_->setText(0, QCoreApplication::translate("Recorder", "Storage", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__storage_);

      combo_setting__storage_ = new QComboBox();
      for (std::string writer : writer_choices)
      {
        combo_setting__storage_->addItem(QString(writer.c_str()));
      }
      ui_recorder_->tree__setting->setItemWidget(tree_setting__storage_, 1, combo_setting__storage_);
    }

    // Setting - Serialization Format
    {
      std::unordered_set<std::string> serialization_choices = rosbag2_cpp::plugins::get_class_plugins<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>();
      auto converters = rosbag2_cpp::plugins::get_class_plugins<rosbag2_cpp::converter_interfaces::SerializationFormatConverter>();
      serialization_choices.insert(converters.begin(), converters.end());

      tree_setting__serialization_format_ = new QTreeWidgetItem();
      tree_setting__serialization_format_->setText(0, QCoreApplication::translate("Recorder", "Serialization Format", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__serialization_format_);

      combo_setting__serialization_format_ = new QComboBox();
      combo_setting__serialization_format_->addItem(QString(""));
      std::string converter_suffix = "_converter";
      auto len = converter_suffix.length();
      for (std::string serialization : serialization_choices)
      {
        auto suffix_pos = serialization.find(converter_suffix);
        if (suffix_pos == std::string::npos || converter_suffix.empty())
        {
          combo_setting__serialization_format_->addItem(QString(serialization.c_str()));
        }
        else
        {
          combo_setting__serialization_format_->addItem(QString(serialization.replace(suffix_pos, len, "").c_str()));
        }
      }
      ui_recorder_->tree__setting->setItemWidget(tree_setting__serialization_format_, 1, combo_setting__serialization_format_);
    }

    // Setting - No Discovery
    {
      tree_setting__no_discovery_ = new QTreeWidgetItem();
      tree_setting__no_discovery_->setText(0, QCoreApplication::translate("Recorder", "No Discovery", nullptr));
      tree_setting__no_discovery_->setCheckState(1, Qt::Unchecked);
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__no_discovery_);
    }

    // Setting - Polling Interval [ms]
    {
      tree_setting__polling_interval_ = new QTreeWidgetItem();
      tree_setting__polling_interval_->setText(0, QCoreApplication::translate("Recorder", "Polling Interval", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__polling_interval_);

      spin_setting__polling_interval_ = new QSpinBox();
      spin_setting__polling_interval_->setMinimum(1);
      spin_setting__polling_interval_->setMaximum(3600000);
      spin_setting__polling_interval_->setValue(100);
      spin_setting__polling_interval_->setSingleStep(10);
      spin_setting__polling_interval_->setSuffix(QString(" ms"));
      spin_setting__polling_interval_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__polling_interval_, 1, spin_setting__polling_interval_);
    }

    // Setting - Max Bag Size [MB]
    {
      tree_setting__max_bag_size_ = new QTreeWidgetItem();
      tree_setting__max_bag_size_->setText(0, QCoreApplication::translate("Recorder", "Max Bag Size", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__max_bag_size_);

      spin_setting__max_bag_size_ = new QSpinBox();
      spin_setting__max_bag_size_->setMinimum(0);
      spin_setting__max_bag_size_->setMaximum(1024 * 1024);
      spin_setting__max_bag_size_->setValue(0);
      spin_setting__max_bag_size_->setSingleStep(1);
      spin_setting__max_bag_size_->setSuffix(QString(" MB"));
      spin_setting__max_bag_size_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__max_bag_size_, 1, spin_setting__max_bag_size_);
    }

    // Setting - Max Bag Duration [s]
    {
      tree_setting__max_bag_duration_ = new QTreeWidgetItem();
      tree_setting__max_bag_duration_->setText(0, QCoreApplication::translate("Recorder", "Max Bag Duration", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__max_bag_duration_);

      spin_setting__max_bag_duration_ = new QSpinBox();
      spin_setting__max_bag_duration_->setMinimum(0);
      spin_setting__max_bag_duration_->setMaximum(24 * 60 * 60);
      spin_setting__max_bag_duration_->setValue(0);
      spin_setting__max_bag_duration_->setSingleStep(10);
      spin_setting__max_bag_duration_->setSuffix(QString(" s"));
      spin_setting__max_bag_duration_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__max_bag_duration_, 1, spin_setting__max_bag_duration_);
    }

    // Setting - Max Cache Size [kB]
    {
      tree_setting__max_cache_size_ = new QTreeWidgetItem();
      tree_setting__max_cache_size_->setText(0, QCoreApplication::translate("Recorder", "Max Cache Size", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__max_cache_size_);

      spin_setting__max_cache_size_ = new QSpinBox();
      spin_setting__max_cache_size_->setMinimum(1);
      spin_setting__max_cache_size_->setMaximum(1024 * 1024);
      spin_setting__max_cache_size_->setValue(1024);
      spin_setting__max_cache_size_->setSingleStep(128);
      spin_setting__max_cache_size_->setSuffix(QString(" kB"));
      spin_setting__max_cache_size_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__max_cache_size_, 1, spin_setting__max_cache_size_);
    }

    // Setting - Compression Mode
    {
      tree_setting__compression_mode_ = new QTreeWidgetItem();
      tree_setting__compression_mode_->setText(0, QCoreApplication::translate("Recorder", "Compression Mode", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__compression_mode_);

      combo_setting__compression_mode_ = new QComboBox();
      combo_setting__compression_mode_->addItem(QString("none"));
      combo_setting__compression_mode_->addItem(QString("file"));
      combo_setting__compression_mode_->addItem(QString("message"));
      ui_recorder_->tree__setting->setItemWidget(tree_setting__compression_mode_, 1, combo_setting__compression_mode_);
    }

    // Setting - Compression Format
    {
      std::unordered_set<std::string> compression_format_choices = rosbag2_cpp::plugins::get_class_plugins<rosbag2_compression::BaseCompressorInterface>();

      tree_setting__compression_format_ = new QTreeWidgetItem();
      tree_setting__compression_format_->setText(0, QCoreApplication::translate("Recorder", "Compression Format", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__compression_format_);

      combo_setting__compression_format_ = new QComboBox();
      combo_setting__compression_format_->addItem(QString(""));
      for (std::string compression_format : compression_format_choices) {
        combo_setting__compression_format_->addItem(QString(compression_format.c_str()));
      }
      ui_recorder_->tree__setting->setItemWidget(tree_setting__compression_format_, 1, combo_setting__compression_format_);
    }

    // Setting - Compression Queue Size
    {
      tree_setting__compression_queue_size_ = new QTreeWidgetItem();
      tree_setting__compression_queue_size_->setText(0, QCoreApplication::translate("Recorder", "Compression Queue Size", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__compression_queue_size_);

      spin_setting__compression_queue_size_ = new QSpinBox();
      spin_setting__compression_queue_size_->setMinimum(1);
      spin_setting__compression_queue_size_->setMaximum(1024);
      spin_setting__compression_queue_size_->setValue(1);
      spin_setting__compression_queue_size_->setSingleStep(1);
      spin_setting__compression_queue_size_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__compression_queue_size_, 1, spin_setting__compression_queue_size_);
    }

    // Setting - Compression Threads
    {
      tree_setting__compression_threads_ = new QTreeWidgetItem();
      tree_setting__compression_threads_->setText(0, QCoreApplication::translate("Recorder", "Compression Threads", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__compression_threads_);

      spin_setting__compression_threads_ = new QSpinBox();
      spin_setting__compression_threads_->setMinimum(1);
      spin_setting__compression_threads_->setMaximum(256);
      spin_setting__compression_threads_->setValue(0);
      spin_setting__compression_threads_->setSingleStep(1);
      spin_setting__compression_threads_->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__compression_threads_, 1, spin_setting__compression_threads_);
    }

    // Setting - Log Level
    {
      tree_setting__log_level_ = new QTreeWidgetItem();
      tree_setting__log_level_->setText(0, QCoreApplication::translate("Recorder", "Log Level", nullptr));
      ui_recorder_->tree__setting->addTopLevelItem(tree_setting__log_level_);

      combo_setting__log_level_ = new QComboBox();
      combo_setting__log_level_->addItem(QString("debug"));
      combo_setting__log_level_->addItem(QString("info"));
      combo_setting__log_level_->addItem(QString("warn"));
      combo_setting__log_level_->addItem(QString("error"));
      combo_setting__log_level_->addItem(QString("fatal"));
      combo_setting__log_level_->setCurrentIndex(1);
      ui_recorder_->tree__setting->setItemWidget(tree_setting__log_level_, 1, combo_setting__log_level_);
    }
  }

  RViz2Bag_Recorder::~RViz2Bag_Recorder() {}

  void RViz2Bag_Recorder::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  void RViz2Bag_Recorder::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void RViz2Bag_Recorder::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
  }

} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Recorder, rviz_common::Panel)
