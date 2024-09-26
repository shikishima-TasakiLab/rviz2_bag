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
#include <rosbag2_storage/storage_options.hpp>
#include <QtWidgets>
#endif

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

protected:
    rclcpp::Node::SharedPtr nh_;
};
    
} // namespace rviz2_bag


#endif // RVIZ2_BAG__PLAYER_HPP_
