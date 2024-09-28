#ifndef RVIZ2_BAG__RECORDER_HPP_
#define RVIZ2_BAG__RECORDER_HPP_

#include "rviz2_bag/visibility_control.hpp"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include <rosbag2_compression/base_compressor_interface.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_serializer.hpp>
#include <rosbag2_cpp/plugins/plugin_utils.hpp>
#include <rosbag2_storage/default_storage_id.hpp>
#include <rosbag2_storage/storage_interfaces/read_write_interface.hpp>
#include <rosbag2_transport/record_options.hpp>

#include <QtCore/QDir>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTreeWidgetItem>

#include "ui_recorder.h"
#endif

namespace Ui
{

    class Recorder;

} // namespace Ui

namespace rviz2_bag
{

    class RViz2Bag_Recorder : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        RViz2Bag_Recorder(QWidget *parent = nullptr);
        ~RViz2Bag_Recorder();

        void onInitialize() override;
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

    protected Q_SLOTS:
        void pbtn__output_dir__clicked();
        void pbtn__record__clicked();
        void pbtn__pause__clicked();
        void pbtn__stop__clicked();
        void pbtn__select_all__clicked();
        void pbtn__deselect_all__clicked();
        void pbtn__topic_refresh__clicked();
        

    protected:
        rclcpp::Node::SharedPtr nh_;
        Ui::Recorder *ui_recorder_;

        std::unique_ptr<rosbag2_transport::RecordOptions> record_options_;

        QTreeWidgetItem *tree_setting__storage_;
        QComboBox *combo_setting__storage_;
        QTreeWidgetItem *tree_setting__serialization_format_;
        QComboBox *combo_setting__serialization_format_;
        QTreeWidgetItem *tree_setting__no_discovery_;
        QTreeWidgetItem *tree_setting__polling_interval_;
        QSpinBox *spin_setting__polling_interval_;
        QTreeWidgetItem *tree_setting__max_bag_size_;
        QSpinBox *spin_setting__max_bag_size_;
        QTreeWidgetItem *tree_setting__max_bag_duration_;
        QSpinBox *spin_setting__max_bag_duration_;
        QTreeWidgetItem *tree_setting__max_cache_size_;
        QSpinBox *spin_setting__max_cache_size_;
        QTreeWidgetItem *tree_setting__compression_mode_;
        QComboBox *combo_setting__compression_mode_;
        QTreeWidgetItem *tree_setting__compression_format_;
        QComboBox *combo_setting__compression_format_;
        QTreeWidgetItem *tree_setting__compression_queue_size_;
        QSpinBox *spin_setting__compression_queue_size_;
        QTreeWidgetItem *tree_setting__compression_threads_;
        QSpinBox *spin_setting__compression_threads_;
        QTreeWidgetItem *tree_setting__log_level_;
        QComboBox *combo_setting__log_level_;

        void list_check_all(Qt::CheckState state);
    };

} // namespace rviz2_bag

#endif // RVIZ2_BAG__RECORDER_HPP_
