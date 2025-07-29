#ifndef TF_SWITCH_NODE_HPP
#define TF_SWITCH_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TfSwitchNode : public rclcpp::Node
{
public:
    TfSwitchNode();

private:
    void publishTransform();
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster broadcaster_;
    geometry_msgs::msg::TransformStamped transform_;
};

#endif // TF_SWITCH_NODE_HPP