#pragma once

#include "motor_control_demo/visibility_control.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <memory>
#include <string>
#include <vector>

namespace motor_control_demo
{
    class MotorControlHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MotorControlHardware);

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        ROS2_MOTOR_CONTROL_DEMO_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        // Parameters for motor control
        double hw_start_sec_;
        double hw_stop_sec_;

        // Store the command for the simulated robot
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
    };
} // namespace motor_control_demo
