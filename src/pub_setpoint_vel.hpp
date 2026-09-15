#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "shared_types.hpp"
#include <iostream>
#include <cmath>

using namespace BT;
using SetPoint = px4_msgs::msg::TrajectorySetpoint;

class PubSetPointVel : public RosTopicPubNode<SetPoint> {
public:
    PubSetPointVel(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
        : RosTopicPubNode<SetPoint>(name, config, params)
    { }

    static PortsList providedPorts() {
        return providedBasicPorts({
            InputPort<Pos>("plate_pos"),
            InputPort<TwistVel>("plate_vel"), 
            InputPort<Pos>("drone_pos"),
            InputPort<Vel>("drone_vel")
        });
    }

    bool setMessage(SetPoint& msg) override {
        Expected<Pos> plate_pos = getInput<Pos>("plate_pos");
        Expected<TwistVel> plate_vel = getInput<TwistVel>("plate_vel");
        Expected<Pos> drone_pos = getInput<Pos>("drone_pos");
        Expected<Vel> drone_vel = getInput<Vel>("drone_vel");

        if (!plate_pos || !plate_vel || !drone_pos || !drone_vel) {
            return false;
        }

        double drone_x = drone_pos.value().x;
        double drone_y = drone_pos.value().y;
        double drone_z = drone_pos.value().z;

        double plat_x = plate_pos.value().x;
        double plat_y = plate_pos.value().y;
        double plat_z = plate_pos.value().z + 0.65; 

        double plat_w = plate_vel.value().angular[2];

        // 1. РОЗРАХУНОК СПРАВЖНЬОЇ ГЛОБАЛЬНОЇ ШВИДКОСТІ
        uint64_t current_time = node_->now().nanoseconds();
        if (first_tick) {
            prev_plat_x = plat_x;
            prev_plat_y = plat_y;
            prev_time = current_time;
            first_tick = false;
        }

        double dt = (current_time - prev_time) / 1e9; 
        if (dt >= 0.05) { 
            global_plat_vx = (plat_x - prev_plat_x) / dt;
            global_plat_vy = (plat_y - prev_plat_y) / dt;
            prev_plat_x = plat_x;
            prev_plat_y = plat_y;
            prev_time = current_time;
        }

        double target_enu_x = 0.0;
        double target_enu_y = 0.0;
        double target_enu_z = 0.0;
        
        // Змінні для Feed-Forward швидкості та прискорення (в ENU)
        double ff_vx = 0.0;
        double ff_vy = 0.0;
        double ff_ax = 0.0; // Прискорення
        double ff_ay = 0.0; 

        if (is_takeoff) {
            target_enu_x = 0.0;
            target_enu_y = 0.0;
            target_enu_z = 2.0;

            if (std::abs(drone_z - 2.0) < 0.2 && std::abs(drone_x) < 0.2 && std::abs(drone_y) < 0.2) {
                is_takeoff = false;
                std::cout << "Takeoff complete! Engaging aggressive intercept." << std::endl;
            }
        } 
        else {
            double drone_vx = drone_vel.value().x;
            double drone_vy = drone_vel.value().y;
            double drone_speed = std::sqrt(drone_vx * drone_vx + drone_vy * drone_vy);
            
            double dist_x = plat_x - drone_x;
            double dist_y = plat_y - drone_y;
            double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

            if (drone_speed < 0.5) {
                target_enu_x = plat_x;
                target_enu_y = plat_y;
                ff_vx = global_plat_vx;
                ff_vy = global_plat_vy;
            } 
            else {
                double t_intercept = distance / drone_speed;
                if (t_intercept > 0.4) t_intercept = 0.4; 

                if (std::abs(plat_w) < 0.01) {
                    target_enu_x = plat_x + global_plat_vx * t_intercept;
                    target_enu_y = plat_y + global_plat_vy * t_intercept;
                    ff_vx = global_plat_vx;
                    ff_vy = global_plat_vy;
                } 
                else {
                    double V = std::sqrt(global_plat_vx * global_plat_vx + global_plat_vy * global_plat_vy);
                    
                    if (V < 0.05) {
                        target_enu_x = plat_x;
                        target_enu_y = plat_y;
                    } else {
                        double theta = std::atan2(global_plat_vy, global_plat_vx);
                        double R = V / plat_w;

                        // Позиція випередження
                        target_enu_x = plat_x + R * (std::sin(theta + plat_w * t_intercept) - std::sin(theta));
                        target_enu_y = plat_y - R * (std::cos(theta + plat_w * t_intercept) - std::cos(theta));
                        
                        // Вектор швидкості на дузі
                        ff_vx = V * std::cos(theta + plat_w * t_intercept);
                        ff_vy = V * std::sin(theta + plat_w * t_intercept);
                        
                        // СЕКРЕТНА ЗБРОЯ 1: Доцентрове прискорення (a = w x v)
                        ff_ax = -plat_w * ff_vy;
                        ff_ay =  plat_w * ff_vx;
                    }
                }
            }
            
            // СЕКРЕТНА ЗБРОЯ 2: Агресивне дотягування (Virtual Spring)
            // Коли дрон вже майже над платформою, ми штучно створюємо додаткову тягу в самий центр
            if (distance < 1.0) {
                target_enu_x += 0.8 * dist_x;
                target_enu_y += 0.8 * dist_y;
            }
            
            target_enu_z = plat_z;
        }

        // КОНВЕРТАЦІЯ ENU -> NED
        msg.timestamp = node_->now().nanoseconds() / 1000ull;
        
        msg.position[0] = static_cast<float>(target_enu_y);
        msg.position[1] = static_cast<float>(target_enu_x);
        msg.position[2] = static_cast<float>(-target_enu_z);

        if (is_takeoff) {
            msg.velocity[0] = NAN; msg.velocity[1] = NAN; msg.velocity[2] = NAN;
            msg.acceleration[0] = NAN; msg.acceleration[1] = NAN; msg.acceleration[2] = NAN;
        } else {
            // Перевід ENU швидкості в NED
            msg.velocity[0] = static_cast<float>(ff_vy); 
            msg.velocity[1] = static_cast<float>(ff_vx); 
            msg.velocity[2] = 0.0f; 
            
            // Перевід ENU прискорення в NED
            msg.acceleration[0] = static_cast<float>(ff_ay);
            msg.acceleration[1] = static_cast<float>(ff_ax);
            msg.acceleration[2] = 0.0f;
        }
        
        double yaw_ned = std::atan2(target_enu_x - drone_x, target_enu_y - drone_y);
        msg.yaw = static_cast<float>(yaw_ned);

        return true;
    }

private:
    bool is_takeoff = true;
    double prev_plat_x = 0.0;
    double prev_plat_y = 0.0;
    uint64_t prev_time = 0;
    bool first_tick = true;
    double global_plat_vx = 0.0;
    double global_plat_vy = 0.0;
};