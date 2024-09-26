#include "rviz2_bag/recorder.hpp"

namespace rviz2_bag
{

RViz2Bag_Recorder::RViz2Bag_Recorder(QWidget *parent)
: rviz_common::Panel(parent)
{
    // init_qt_panel();
}

RViz2Bag_Recorder::~RViz2Bag_Recorder() {}

void RViz2Bag_Recorder::onInitialize()
{
    // init_ros_node();
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
