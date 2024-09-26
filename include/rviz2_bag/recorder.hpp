#ifndef RVIZ2_BAG__RECORDER_HPP_
#define RVIZ2_BAG__RECORDER_HPP_

#include "rviz2_bag/visibility_control.hpp"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#include <QtWidgets>
#endif

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

protected:
    rclcpp::Node::SharedPtr nh_;
};
    
} // namespace rviz2_bag


#endif // RVIZ2_BAG__RECORDER_HPP_
