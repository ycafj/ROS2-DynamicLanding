#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "pub_vehiclecommand.hpp"
#include "pub_ovehcommand.hpp"
#include "move_platform.hpp"
#include "pub_offboard.hpp"
#include "pub_setpoint_vel.hpp"
#include "get_pos_vel.hpp"
#include "get_gz_pos_vel.hpp"
#include "fly.hpp"
#include "ned-enu_pos_vel.hpp"
#include <thread>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace BT;

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    BehaviorTreeFactory factory;
    auto ros_node = std::make_shared<rclcpp::Node>("bt_px4_node");
    BT::RosNodeParams params;
    params.nh = ros_node;
    factory.registerNodeType<MovePlatform>("MovePlatform", params);
    factory.registerNodeType<PubVehicleCommand>("PubVehicleCommand", params);
    factory.registerNodeType<SetOffboardMode>("SetOffboardMode", params);  
    factory.registerNodeType<PubSetPointVel>("PubSetPointVel", params);
    factory.registerNodeType<PubSetPointVel>("PubSetPoint", params);
    factory.registerNodeType<Fly>("Fly");
    factory.registerNodeType<GetPlatformPosVel>("GetPlatePosVel", params);
    factory.registerNodeType<GetPosVel>("GetPosVel", params);
    factory.registerNodeType<PubOffboardControlMode>("PubOffboard", params);
    factory.registerNodeType<NedToEnuPosVel>("NEDtoENUPosVel");
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("project1");
    std::string bt_xml_path = package_share_directory + "/btcpp/MoveToSetPoint.xml";
    factory.registerBehaviorTreeFromFile(bt_xml_path);
    bt_xml_path = package_share_directory + "/btcpp/proj1.xml";
    factory.registerBehaviorTreeFromFile(bt_xml_path);
    auto tree  = factory.createTree("proj1");
    Groot2Publisher publisher(tree);
    while(rclcpp::ok()) {
        BT::NodeStatus status = tree.tickExactlyOnce();
        if(status == BT::NodeStatus::SUCCESS){
            break;
        }
        rclcpp::spin_some(ros_node);
    }
    
    return 0;
};