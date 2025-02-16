// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_HPP
#define ROS2_CONTROL_HPP

#include <memory>
#include <array>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "webots/motor.h"
#include "webots/position_sensor.h"
#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"

namespace webots_ros2_control {
  struct Joint {
    double positionCommand;
    double position;
    double velocityCommand;
    double velocity;
    double effortCommand;
    double acceleration;
    bool controlPosition;
    bool controlVelocity;
    bool controlEffort;
    std::string name;
    WbDeviceTag motor;
    WbDeviceTag sensor;
  };

  struct Imu {
    std::string name;
    WbDeviceTag inertial;
    WbDeviceTag gyro;
    WbDeviceTag accelerometer;

    std::vector<std::string> state_interfaces;
    std::array<double, 10> imu_sensor_data = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  };

  struct ForceTorqueSensor {
    std::string name;
    WbDeviceTag touch_sensor;

    std::vector<std::string> state_interfaces;
    std::array<double, 6> force_torque_sensor_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  };

  class Ros2ControlSystem : public Ros2ControlSystemInterface {
  public:
    Ros2ControlSystem();
    void init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

  private:
    static constexpr auto imu_type_name = "IMU";
    static constexpr int imu_default_update_rate = 500;

    static constexpr auto ft_sensor_type_name = "FTSensor";
    static constexpr int ft_sensor_default_update_rate = 500;

    const std::unordered_map<std::string, size_t> imu_interface_name_map = {
      {"orientation.x", 0},
      {"orientation.y", 1},
      {"orientation.z", 2},
      {"orientation.w", 3},
      {"angular_velocity.x", 4},
      {"angular_velocity.y", 5},
      {"angular_velocity.z", 6},
      {"linear_acceleration.x", 7},
      {"linear_acceleration.y", 8},
      {"linear_acceleration.z", 9},
    };

    const std::unordered_map<std::string, size_t> ft_interface_name_map = {
      {"force.x", 0},
      {"force.y", 1},
      {"force.z", 2},
      {"torque.x", 3},
      {"torque.y", 4},
      {"torque.z", 5},
    };

    void registerSensors(const hardware_interface::HardwareInfo & hardware_info);

    webots_ros2_driver::WebotsNode *mNode;
    std::vector<Joint> mJoints;
    std::vector<Imu> mImus;
    std::vector<ForceTorqueSensor> mForceTorqueSensors;
  };
}  // namespace webots_ros2_control

#endif
