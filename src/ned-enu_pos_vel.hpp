#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "behaviortree_cpp/action_node.h"
#include "shared_types.hpp"
#include <iostream>

using namespace BT;
using Position = px4_msgs::msg::VehicleLocalPosition;




class NedToEnuPosVel : public SyncActionNode {
public:
    NedToEnuPosVel(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    { 
    }

    static PortsList providedPorts(){
        return {
            InputPort<Pos>("nedpos"), 
            InputPort<Vel>("nedvel"), 
            OutputPort<Pos>("enupos"),
            OutputPort<Vel>("enuvel")
        };
    }

    NodeStatus tick() override 
    {
        Expected<Pos> nedpos = getInput<Pos>("nedpos");
        Expected<Vel> nedvel = getInput<Vel>("nedvel");
        if (!nedpos || !nedvel) {
            std::cout<<"Failed to get input port: " << (nedpos ? nedvel.error() : nedpos.error()) << std::endl;
            return NodeStatus::FAILURE;
        }
        setOutput("enupos", Pos{nedpos.value().y, nedpos.value().x, -nedpos.value().z});
        setOutput("enuvel", Vel{nedvel.value().y, nedvel.value().x, -nedvel.value().z});
        std::cout<<"Success NedEnu"<<std::endl;     
        return NodeStatus::SUCCESS;
    }


};