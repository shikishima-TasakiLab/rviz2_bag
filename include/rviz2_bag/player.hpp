#ifndef RVIZ2_BAG__PLAYER_HPP_
#define RVIZ2_BAG__PLAYER_HPP_

#include "rviz2_bag/visibility_control.hpp"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <rosbag2_transport/play_options.hpp>
#include <rosbag2_transport/player.hpp>
#include "rosbag2_transport/reader_writer_factory.hpp"
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <QtWidgets/QFileDialog>

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
        void pbtn__rosbag_open__callback();
        void pbtn__select_all__callback();
        void pbtn__deselect_all__callback();
        void pbtn__rosbag_play__callback();
        void pbtn__rosbag_stop__callback();
        void pbtn__rosbag_pause__callback();
        void pbtn__backward__callback();
        void pbtn__play_next__callback();

    protected:
        rclcpp::Node::SharedPtr nh_;
        Ui::Player *ui_player_;

        std::unique_ptr<rosbag2_storage::StorageOptions> storage_options_;
        std::unique_ptr<rosbag2_transport::Player> bag_player_;

        void list_check_all(Qt::CheckState state);
    };

} // namespace rviz2_bag

#endif // RVIZ2_BAG__PLAYER_HPP_
