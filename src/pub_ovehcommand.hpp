#pragma once
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

using namespace BT;
using VehicleCommand = px4_msgs::msg::VehicleCommand;

class SetOffboardMode : public RosTopicPubNode<VehicleCommand>{
public:
    SetOffboardMode(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosTopicPubNode<VehicleCommand>(name, config, params) {}

    static PortsList providedPorts(){
        return providedBasicPorts({});
    }

    bool setMessage(VehicleCommand& msg) override {
        msg.param1 = 1.0f; 
        msg.param2 = 6.0f; // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = node_->now().nanoseconds() / 1000ull;
        return true;
    }
};