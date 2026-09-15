#pragma once
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "shared_types.hpp"
using namespace BT;
using PlatformPosition = nav_msgs::msg::Odometry;

class GetPlatformPosVel : public RosTopicSubNode<PlatformPosition> {
public:
    GetPlatformPosVel(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosTopicSubNode<PlatformPosition>(name, config, params)
    {}

    static PortsList providedPorts(){
    return providedBasicPorts({
        OutputPort<Pos>("pos"), 
        OutputPort<TwistVel>("vel")
    });
}

NodeStatus onTick(const std::shared_ptr<PlatformPosition>& last_msg) override 
{
    if (last_msg) 
    {
        double pos_x = last_msg->pose.pose.position.x;
        double pos_y = last_msg->pose.pose.position.y;
        double pos_z = last_msg->pose.pose.position.z;
        TwistVel twist_vel;
        twist_vel.linear[0] = static_cast<float>(last_msg->twist.twist.linear.x);
        twist_vel.linear[1] = static_cast<float>(last_msg->twist.twist.linear.y);
        twist_vel.linear[2] = static_cast<float>(last_msg->twist.twist.linear.z);

        twist_vel.angular[0] = static_cast<float>(last_msg->twist.twist.angular.x);
        twist_vel.angular[1] = static_cast<float>(last_msg->twist.twist.angular.y);
        twist_vel.angular[2] = static_cast<float>(last_msg->twist.twist.angular.z);
        setOutput("pos", Pos{static_cast<double>(pos_x), static_cast<double>(pos_y), static_cast<double>(pos_z)});
        setOutput("vel", twist_vel);

        std::cout << "Success Get PosVel" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}
    bool latchLastMessage() const override{
        return true;
    }
};