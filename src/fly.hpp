#include "behaviortree_cpp/bt_factory.h"
#include "shared_types.hpp"
#include <cmath>
#include <iostream>

using namespace BT;

class Fly : public ActionNodeBase {
public:
    Fly(const std::string& name, const NodeConfig& config)
        : ActionNodeBase(name, config)
    { }

    static PortsList providedPorts() {
        return {
            InputPort<Pos>("plate_pos"),
            InputPort<Pos>("drone_pos")
        };
    }

    NodeStatus tick() override {
        // std::cout << "Виконується Fly..." << std::endl;
        Expected<Pos> plate_pos = getInput<Pos>("plate_pos");
        Expected<Pos> drone_pos = getInput<Pos>("drone_pos");

        if (!plate_pos || !drone_pos) {
            return NodeStatus::RUNNING;
        }
        double cur_x = drone_pos.value().x;
        double cur_y = drone_pos.value().y;
        double cur_z = drone_pos.value().z;
        double plat_x = plate_pos.value().x;
        double plat_y = plate_pos.value().y;
        double plat_z = plate_pos.value().z + 0.62;

        double err_x = std::abs(plat_x - cur_x);
        double err_y = std::abs(plat_y - cur_y);
        double err_z = std::abs(plat_z - cur_z);

        if (err_x < 0.05 && err_y < 0.05 && err_z < 0.05) {
            std::cout << "===========================" << std::endl;
            std::cout << "      ЦІЛЬ ДОСЯГНУТА       " << std::endl;
            std::cout << "===========================" << std::endl;
            return NodeStatus::SUCCESS;
        }
        else {
            std::cout << "[INFO] Поточна позиція: (" << cur_x << ", " << cur_y << ", " << cur_z << ")" << std::endl;
            std::cout << "[INFO] До цілі залишилось: " << err_x << " по X, " << err_y << " по Y, " << err_z << " по Z" << std::endl;
            return NodeStatus::RUNNING;
        }
    }

    void halt() override {
    }
};