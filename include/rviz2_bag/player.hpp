#ifndef RVIZ2_BAG__PLAYER_HPP_
#define RVIZ2_BAG__PLAYER_HPP_

#include "rviz2_bag/visibility_control.hpp"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <QtWidgets/QFileDialog>

#include "rviz2_bag/rosbag2_transport/player.hpp"
#include "ui_player.h"
#endif

namespace Ui
{

    class Player;

} // namespace Ui

namespace rviz2_bag
{

    class RViz2Bag_Player : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        RViz2Bag_Player(QWidget *parent = nullptr);
        ~RViz2Bag_Player();

        void onInitialize() override;
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

    protected Q_SLOTS:
        void dspin__rosbag_rate__valueChanged(double value);
        void dspin__rosbag_elapsed_time__valueChanged(double value);
        void dspin__rosbag_elapsed_time__editingFinished();
        void hsld__rosbag_elapsed_time__valueChanged(int value);
        void hsld__rosbag_elapsed_time__sliderReleased();
        void pbtn__rosbag_open__callback();
        void pbtn__select_all__callback();
        void pbtn__deselect_all__callback();
        void pbtn__rosbag_play__callback();
        void pbtn__rosbag_stop__callback();
        void pbtn__rosbag_pause__callback();
        void pbtn__backward__callback();
        void pbtn__play_next__callback();
        void update_elapsed_time__callback(rcutils_time_point_value_t time_point);

    protected:
        rclcpp::Node::SharedPtr nh_;
        Ui::Player *ui_player_;

        std::unique_ptr<rosbag2_storage::StorageOptions> storage_options_;
        std::unique_ptr<rosbag2_storage::BagMetadata> metadata_;

        std::shared_ptr<rosbag2_transport::Player> bag_player_;
        std::unique_ptr<std::thread> spin_thread_;
        std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;

        int hsld__rosbag_elapsed_time__value__pause_{0};

        void list_check_all(Qt::CheckState state);
        bool play();
        void stop();
    };

} // namespace rviz2_bag

#endif // RVIZ2_BAG__PLAYER_HPP_
