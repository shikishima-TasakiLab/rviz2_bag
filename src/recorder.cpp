#include "rviz2_bag/recorder.hpp"

#include <QtWidgets/QStyledItemDelegate>

namespace rviz2_bag
{
  class UneditableDelegate : public QStyledItemDelegate
  {
  public:
    UneditableDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}
    virtual QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
    {
      return nullptr;
    }
  };

  RViz2Bag_Recorder::RViz2Bag_Recorder(QWidget *parent)
      : rviz_common::Panel(parent), ui_recorder_(new Ui::Recorder())
  {
    ui_recorder_->setupUi(this);

    // Setting
    {
      ui_recorder_->tree__setting->setItemDelegateForColumn(0, new UneditableDelegate(this));

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
        QString converter_suffix("_converter");
        for (std::string serialization : serialization_choices)
        {
          combo_setting__serialization_format_->addItem(QString(serialization.c_str()).replace(converter_suffix, QString("")));
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
        spin_setting__polling_interval_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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
        spin_setting__max_bag_size_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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
        spin_setting__max_bag_duration_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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
        spin_setting__max_cache_size_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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
        for (std::string compression_format : compression_format_choices)
        {
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
        spin_setting__compression_queue_size_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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
        spin_setting__compression_threads_->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
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

      // Setting - Name Delimiter
      {
        tree_setting__name_delimiter_ = new QTreeWidgetItem();
        tree_setting__name_delimiter_->setText(0, QCoreApplication::translate("Recorder", "Name Delimiter", nullptr));
        tree_setting__name_delimiter_->setText(1, QString("_"));
        tree_setting__name_delimiter_->setFlags(tree_setting__name_delimiter_->flags() | Qt::ItemIsEditable);
        ui_recorder_->tree__setting->addTopLevelItem(tree_setting__name_delimiter_);
      }
    }

    connect(
        ui_recorder_->pbtn__output_dir,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__output_dir__clicked);
    connect(
        ui_recorder_->pbtn__record,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__record__clicked);
    connect(
        ui_recorder_->pbtn__pause,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__pause__clicked);
    connect(
        ui_recorder_->pbtn__stop,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__stop__clicked);
    connect(
        ui_recorder_->pbtn__select_all,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__select_all__clicked);
    connect(
        ui_recorder_->pbtn__deselect_all,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__deselect_all__clicked);
    connect(
        ui_recorder_->pbtn__topic_refresh,
        &QPushButton::clicked,
        this,
        &RViz2Bag_Recorder::pbtn__topic_refresh__clicked);
    connect(
        ui_recorder_->tree__setting,
        &QTreeWidget::itemChanged,
        this,
        &RViz2Bag_Recorder::tree__setting__itemChanged);
  }

  RViz2Bag_Recorder::~RViz2Bag_Recorder()
  {
    stop();
  }

  void RViz2Bag_Recorder::onInitialize()
  {
    nh_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  }

  void RViz2Bag_Recorder::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);

    rviz_common::Config topics = config.mapMakeChild("topics");
    int list_count = ui_recorder_->list__topics->topLevelItemCount();
    for (int i = 0; i < list_count; i++)
    {
      QTreeWidgetItem *item = ui_recorder_->list__topics->topLevelItem(i);
      rviz_common::Config topic = topics.mapMakeChild(item->text(0));
      topic.mapSetValue("checked", item->checkState(0) == Qt::Checked);
      topic.mapSetValue("type", item->text(1));
    }

    rviz_common::Config settings = config.mapMakeChild("settings");
    settings.mapSetValue("Name_Prefix", ui_recorder_->ledit__name_prefix->text());
    settings.mapSetValue("Name_Timestamp", ui_recorder_->check__name_timestamp->checkState() == Qt::Checked);
    settings.mapSetValue("Name_Suffix", ui_recorder_->ledit__name_suffix->text());
    settings.mapSetValue("use_Sim_Time", ui_recorder_->check__use_sim_time->checkState() == Qt::Checked);
    settings.mapSetValue("Storage", combo_setting__storage_->currentText());
    settings.mapSetValue("Serialization_Format", combo_setting__serialization_format_->currentText());
    settings.mapSetValue("No_Discovery", tree_setting__no_discovery_->checkState(1) == Qt::Checked);
    settings.mapSetValue("Polling_Interval__ms", spin_setting__polling_interval_->value());
    settings.mapSetValue("Max_Bag_Size__MB", spin_setting__max_bag_size_->value());
    settings.mapSetValue("Max_Bag_Duration__s", spin_setting__max_bag_duration_->value());
    settings.mapSetValue("Max_Cache_Size__kB", spin_setting__max_cache_size_->value());
    settings.mapSetValue("Compression_Mode", combo_setting__compression_mode_->currentText());
    settings.mapSetValue("Compression_Format", combo_setting__compression_format_->currentText());
    settings.mapSetValue("Compression_Queue_Size", spin_setting__compression_queue_size_->value());
    settings.mapSetValue("Compression_Threads", spin_setting__compression_threads_->value());
    settings.mapSetValue("Log_Level", combo_setting__log_level_->currentText());
    settings.mapSetValue("Name_Delimiter", tree_setting__name_delimiter_->text(1));
  }

  void RViz2Bag_Recorder::load(const rviz_common::Config &config)
  {
    rviz_common::Panel::load(config);
  }

  void RViz2Bag_Recorder::pbtn__output_dir__clicked()
  {
    QString str_dir = QFileDialog::getExistingDirectory(this, tr("Open ROSBAG"), QDir::homePath(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    QDir dir(str_dir);
    if (dir.exists() == false)
      return;

    ui_recorder_->ledit__output_dir->setText(str_dir);

    ui_recorder_->list__topics->setEnabled(true);
    ui_recorder_->check__use_sim_time->setEnabled(true);
    ui_recorder_->pbtn__record->setEnabled(true);
    ui_recorder_->pbtn__pause->setEnabled(false);
    ui_recorder_->pbtn__stop->setEnabled(false);
    ui_recorder_->pbtn__select_all->setEnabled(true);
    ui_recorder_->pbtn__deselect_all->setEnabled(true);
    ui_recorder_->pbtn__topic_refresh->setEnabled(true);
  }

  void RViz2Bag_Recorder::pbtn__record__clicked()
  {
    if (bag_recorder_ == nullptr)
    {
      if (record() == false)
        return;
    }
    else
    {
      bag_recorder_->resume();
    }

    ui_recorder_->list__topics->setEnabled(false);
    ui_recorder_->check__use_sim_time->setEnabled(false);
    ui_recorder_->pbtn__record->setEnabled(false);
    ui_recorder_->pbtn__pause->setEnabled(true);
    ui_recorder_->pbtn__stop->setEnabled(true);
    ui_recorder_->pbtn__select_all->setEnabled(false);
    ui_recorder_->pbtn__deselect_all->setEnabled(false);
    ui_recorder_->pbtn__topic_refresh->setEnabled(false);
  }

  void RViz2Bag_Recorder::pbtn__pause__clicked()
  {
    bag_recorder_->pause();

    ui_recorder_->list__topics->setEnabled(false);
    ui_recorder_->check__use_sim_time->setEnabled(false);
    ui_recorder_->pbtn__record->setEnabled(true);
    ui_recorder_->pbtn__pause->setEnabled(false);
    ui_recorder_->pbtn__stop->setEnabled(true);
    ui_recorder_->pbtn__select_all->setEnabled(false);
    ui_recorder_->pbtn__deselect_all->setEnabled(false);
    ui_recorder_->pbtn__topic_refresh->setEnabled(false);
  }

  void RViz2Bag_Recorder::pbtn__stop__clicked()
  {
    stop();

    ui_recorder_->list__topics->setEnabled(true);
    ui_recorder_->check__use_sim_time->setEnabled(true);
    ui_recorder_->pbtn__record->setEnabled(true);
    ui_recorder_->pbtn__pause->setEnabled(false);
    ui_recorder_->pbtn__stop->setEnabled(false);
    ui_recorder_->pbtn__select_all->setEnabled(true);
    ui_recorder_->pbtn__deselect_all->setEnabled(true);
    ui_recorder_->pbtn__topic_refresh->setEnabled(true);
  }

  void RViz2Bag_Recorder::pbtn__select_all__clicked()
  {
    list_check_all(Qt::Checked);
  }

  void RViz2Bag_Recorder::pbtn__deselect_all__clicked()
  {
    list_check_all(Qt::Unchecked);
  }

  void RViz2Bag_Recorder::pbtn__topic_refresh__clicked()
  {
    int list_count = ui_recorder_->list__topics->topLevelItemCount();
    for (int i = 0; i < list_count; i++)
    {
      QTreeWidgetItem *item = ui_recorder_->list__topics->takeTopLevelItem(0);
      delete item;
    }

    std::map<std::string, std::vector<std::string>> topic_infos = nh_->get_topic_names_and_types();
    for (const auto &topic_itr : topic_infos)
    {
      std::string topic_name = topic_itr.first;
      std::vector<std::string> topic_types = topic_itr.second;
      for (const auto &topic_type : topic_types)
      {
        QTreeWidgetItem *item = new QTreeWidgetItem();
        item->setText(0, QString(topic_name.c_str()));
        item->setText(1, QString(topic_type.c_str()));
        item->setCheckState(0, Qt::Checked);
        ui_recorder_->list__topics->addTopLevelItem(item);
      }
    }
  }

  void RViz2Bag_Recorder::tree__setting__itemChanged(QTreeWidgetItem *item, int column)
  {
    if ((item == tree_setting__name_delimiter_) && (column == 1))
    {
      item->setText(
          column,
          item->text(column)
              .replace("\\", "")
              .replace("/", "")
              .replace(":", "")
              .replace("*", "")
              .replace("?", "")
              .replace("\"", "")
              .replace("<", "")
              .replace(">", "")
              .replace("|", ""));
    }
  }

  void RViz2Bag_Recorder::list_check_all(Qt::CheckState state)
  {
    int list_count = ui_recorder_->list__topics->topLevelItemCount();
    for (int i = 0; i < list_count; i++)
    {
      QTreeWidgetItem *item = ui_recorder_->list__topics->topLevelItem(i);
      item->setCheckState(0, state);
    }
  }

  bool RViz2Bag_Recorder::record()
  {
    // Storage Options
    {
      storage_options_ = std::make_unique<rosbag2_storage::StorageOptions>();

      // storage_options_->uri
      {
        QString uri = ui_recorder_->ledit__name_prefix->text(); // prefix
        if (ui_recorder_->check__name_timestamp->checkState() == Qt::Checked)
        {
          if (uri.isEmpty() == false)
            uri += tree_setting__name_delimiter_->text(1);

          QDateTime datetime = QDateTime::currentDateTime();
          uri += datetime.toString("yyyy_MM_dd-hh_mm_ss");
        }
        QString suffix = ui_recorder_->ledit__name_suffix->text();
        if (suffix.isEmpty() == false)
        {
          if (uri.isEmpty() == false)
            uri += tree_setting__name_delimiter_->text(1);

          uri += ui_recorder_->ledit__name_suffix->text();
        }

        QDir output_dir(ui_recorder_->ledit__output_dir->text());
        QDir rosbag_dir(output_dir.filePath(uri));
        storage_options_->uri = rosbag_dir.path().toLocal8Bit().constData();
        if (rosbag_dir.exists() == true)
        {
          RCLCPP_ERROR_STREAM(
              nh_->get_logger(),
              "[rviz2_bag] Output folder '" << storage_options_->uri << "' already exists.");
          return false;
        }

        RCLCPP_DEBUG_STREAM(
            nh_->get_logger(),
            "[rviz2_bag] " << storage_options_->uri);
      }

      storage_options_->storage_id = combo_setting__storage_->currentText().toLocal8Bit().constData();
      storage_options_->max_bagfile_size = spin_setting__max_bag_size_->value();
      storage_options_->max_bagfile_duration = spin_setting__max_bag_duration_->value();
      storage_options_->max_cache_size = spin_setting__max_cache_size_->value();
      storage_options_->storage_preset_profile = "";
      storage_options_->storage_config_uri = "";
      storage_options_->snapshot_mode = false;
    }

    // Record Options
    {
      record_options_ = std::make_unique<rosbag2_transport::RecordOptions>();

      record_options_->all = false;
      record_options_->is_discovery_disabled = (tree_setting__no_discovery_->checkState(1) == Qt::Checked);
      // record_options_->topics
      {
        int list_count = ui_recorder_->list__topics->topLevelItemCount();
        for (int i = 0; i < list_count; i++)
        {
          QTreeWidgetItem *item = ui_recorder_->list__topics->topLevelItem(i);
          if (item->checkState(0) == Qt::Checked)
          {
            std::string topic = item->text(0).toLocal8Bit().constData();
            record_options_->topics.push_back(topic);
            RCLCPP_DEBUG_STREAM(
                nh_->get_logger(),
                "[rviz2_bag] " << topic);
          }
        }

        if (record_options_->topics.size() < 1)
        {
          RCLCPP_ERROR_STREAM(
              nh_->get_logger(),
              "[rviz2_bag] "
              "Invalid choice: Must specify topic(s).");
          return false;
        }
      }
      // record_options_->rmw_serialization_format
      {
        record_options_->rmw_serialization_format = combo_setting__serialization_format_->currentText().toLocal8Bit().constData();
        if (record_options_->rmw_serialization_format.empty())
          record_options_->rmw_serialization_format = std::string(rmw_get_serialization_format());
      }

      record_options_->topic_polling_interval = std::chrono::milliseconds(spin_setting__polling_interval_->value());
      record_options_->regex = "";
      record_options_->exclude = "";
      record_options_->node_prefix = "";
      // record_options_->compression_mode
      // record_options_->compression_format
      // record_options_->compression_queue_size
      // record_options_->compression_threads
      {
        QString compression_mode = combo_setting__compression_mode_->currentText();
        QString compression_format = combo_setting__compression_format_->currentText();
        if ((compression_mode == "none") && (compression_format.isEmpty() == false))
        {
          RCLCPP_ERROR_STREAM(
              nh_->get_logger(),
              "[rviz2_bag] "
              "Invalid choice "
              "(Compression Mode & Compression Format): "
              "Cannot specify compression format without a compression mode.");
          return false;
        }

        if ((storage_options_->storage_id == "mcap") && (compression_mode == "message"))
        {
          RCLCPP_ERROR_STREAM(
              nh_->get_logger(),
              "[rviz2_bag] "
              "Invalid choice "
              "(Storage & Compression Mode): "
              "compression_mode 'message' is not supported by the MCAP storage plugin. You can enable chunk compression by setting `compression: 'Zstd'` in storage config.");
          return false;
        }

        int compression_queue_size = spin_setting__compression_queue_size_->value();
        if (compression_queue_size < 0)
        {
          RCLCPP_ERROR_STREAM(
              nh_->get_logger(),
              "[rviz2_bag] "
              "Compression queue size must be at least 0.");
          return false;
        }

        record_options_->compression_mode = compression_mode.toUpper().toLocal8Bit().constData();
        record_options_->compression_format = compression_format.toLocal8Bit().constData();
        record_options_->compression_queue_size = compression_queue_size;
        record_options_->compression_threads = spin_setting__compression_threads_->value();
      }
      record_options_->topic_qos_profile_overrides = {};
      record_options_->include_hidden_topics = false;
      record_options_->include_unpublished_topics = false;
      record_options_->ignore_leaf_topics = false;
      record_options_->start_paused = false;
      record_options_->use_sim_time = (ui_recorder_->check__use_sim_time->checkState() == Qt::Checked);

      if (record_options_->is_discovery_disabled && record_options_->use_sim_time)
      {
        RCLCPP_ERROR_STREAM(
            nh_->get_logger(),
            "[rviz2_bag] 'use Sim Time' and 'No Discovery' both set, but are incompatible settings. "
            "The /clock topic needs to be discovered to record with sim time.");
        return false;
      }
    }

    auto writer = rosbag2_transport::ReaderWriterFactory::make_writer(*record_options_);
    bag_recorder_ = std::make_shared<rosbag2_transport::Recorder>(
        std::move(writer), *storage_options_, *record_options_, nh_);

    // spin_thread_ = std::make_unique<std::thread>([this]()
    //                                              { this->bag_recorder_->record(); });
    bag_recorder_->record();

    return true;
  }

  void RViz2Bag_Recorder::stop()
  {
    if (bag_recorder_ == nullptr)
    {
    }
    else
    {
      bag_recorder_->stop();
      bag_recorder_.reset();
    }
  }

} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Recorder, rviz_common::Panel)
