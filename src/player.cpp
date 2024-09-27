#include "rviz2_bag/player.hpp"

namespace rviz2_bag
{

RViz2Bag_Player::RViz2Bag_Player(QWidget *parent)
: rviz_common::Panel(parent)
, ui_player_(new Ui::Player())
{
    ui_player_->setupUi(this);
    // init_qt_panel();
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
    
} // namespace rviz2_bag

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_bag::RViz2Bag_Player, rviz_common::Panel)
