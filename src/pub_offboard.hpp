#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include <iostream>
using namespace BT;
using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
class PubOffboardControlMode : public RosTopicPubNode<OffboardControlMode>{
public:
    PubOffboardControlMode(const std::string& name,const NodeConfig& config, const RosNodeParams& params)
        :RosTopicPubNode<OffboardControlMode>(name, config, params)
    { }
    static PortsList providedPorts(){
        return providedBasicPorts({});
    }
    bool setMessage(OffboardControlMode& msg) override{
        msg.timestamp = node_->now().nanoseconds() / 1000ull;
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false; 
        msg.thrust_and_torque = false;
        msg.direct_actuator = false;
        return true;
    }
};