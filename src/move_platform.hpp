#pragma once
 #include "behaviortree_ros2/bt_topic_pub_node.hpp"
 #include "geometry_msgs/msg/twist.hpp"
 #include <cmath>

 using namespace BT;
 class MovePlatform : public RosTopicPubNode<geometry_msgs::msg::Twist>
 {
public:
    MovePlatform(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
    {}
 
    static PortsList providedPorts()
    {
        return providedBasicPorts({});
    }
 
    bool setMessage(geometry_msgs::msg::Twist& msg) override
    {       
        msg.linear.x = 0.0; 
        msg.linear.y = 1.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = -0.5;
        return true;
    }
};