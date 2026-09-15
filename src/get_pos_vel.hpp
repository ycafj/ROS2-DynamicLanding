#pragma once
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "shared_types.hpp"
using namespace BT;
using Position = px4_msgs::msg::VehicleLocalPosition;

class GetPosVel : public RosTopicSubNode<Position> {
private:
    // Створюємо власного підписника та змінну для повідомлень
    rclcpp::Subscription<Position>::SharedPtr px4_sub_;
    std::shared_ptr<Position> best_effort_msg_;

public:
    GetPosVel(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosTopicSubNode<Position>(name, config, params)
    {
        // Базовий клас створить свій Reliable підписник (який нічого не зловить),
        // а ми створюємо свій Best Effort підписник, який реально працюватиме з PX4:
        px4_sub_ = params.nh.lock()->create_subscription<Position>(
            "/fmu/out/vehicle_local_position_v1",
            rclcpp::SensorDataQoS(), 
            [this](const Position::SharedPtr msg) {
                best_effort_msg_ = msg; // Зберігаємо повідомлення сюди
            }
        );
    }

    static PortsList providedPorts(){
        return providedBasicPorts({
            OutputPort<Pos>("pos"), 
            OutputPort<Vel>("vel")
        });
    }

    // Зверніть увагу: базовий last_msg ми просто ігноруємо
    NodeStatus onTick(const std::shared_ptr<Position>& /*last_msg*/) override 
    {
        // Перевіряємо НАШУ змінну
        if (best_effort_msg_) 
        {
            setOutput("pos", Pos{best_effort_msg_->x, best_effort_msg_->y, best_effort_msg_->z});
            setOutput("vel", Vel{best_effort_msg_->vx, best_effort_msg_->vy, best_effort_msg_->vz});
            std::cout << "Success Get PX4Pos (Best Effort)" << std::endl;    
            return NodeStatus::SUCCESS;
        }
        
        return NodeStatus::FAILURE;
    }

    bool latchLastMessage() const override {
        return true;
    }
};