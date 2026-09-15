#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <iostream>
using namespace BT;
using VehicleCommand = px4_msgs::msg::VehicleCommand;
class PubVehicleCommand : public RosTopicPubNode<VehicleCommand>{
public:
    PubVehicleCommand(const std::string& name,const NodeConfig& config, const RosNodeParams& params)
        :RosTopicPubNode<VehicleCommand>(name, config, params)
    { }
    static PortsList providedPorts(){
        return providedBasicPorts({InputPort<float>("param1"),InputPort<int>("command")});
    }
    bool setMessage(VehicleCommand& msg) override{
        Expected<float> param1 = getInput<float>("param1");
        Expected<int> command = getInput<int>("command");
        if (!param1 || !command) {
            std::cout<<"Failed to get input port: " << param1.error() << " or " << command.error() << std::endl;
            return false;
        }
        else{
            msg.param1 = param1.value();
            msg.command = command.value();
        }
        msg.param2 = 0.0f; 
        if (msg.command == 400 && msg.param1 == 1.0) {
        msg.param2 = 21196.0; 
        }
	    msg.target_system = 1;
	    msg.target_component = 1;
	    msg.source_system = 1;
	    msg.source_component = 1;
	    msg.from_external = true;
	    msg.timestamp = node_->now().nanoseconds() / 1000ull;
        std::cout<<"Publishing VehicleCommand:command="<<command.value()<<std::endl;
        return true;
    }
};