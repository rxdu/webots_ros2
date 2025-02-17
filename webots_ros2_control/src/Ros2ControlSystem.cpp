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

#include "webots_ros2_control/Ros2ControlSystem.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include <webots/device.h>
#include <webots/robot.h>
#include <webots/accelerometer.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/touch_sensor.h>

namespace webots_ros2_control {
  Ros2ControlSystem::Ros2ControlSystem() {
    mNode = NULL;
  }
  void Ros2ControlSystem::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) {
    mNode = node;
    for (hardware_interface::ComponentInfo component : info.joints) {
      Joint joint;
      joint.name = component.name;

      WbDeviceTag device = wb_robot_get_device(joint.name.c_str());
      WbNodeType type = wb_device_get_node_type(device);
      joint.motor =
        (type == WB_NODE_LINEAR_MOTOR || type == WB_NODE_ROTATIONAL_MOTOR) ? device : wb_position_sensor_get_motor(device);
      device = (component.parameters.count("sensor") == 0) ? wb_robot_get_device(joint.name.c_str()) :
                                                             wb_robot_get_device(component.parameters.at("sensor").c_str());
      type = wb_device_get_node_type(device);
      joint.sensor = (type == WB_NODE_POSITION_SENSOR) ? device : wb_motor_get_position_sensor(device);

      if (joint.sensor)
        wb_position_sensor_enable(joint.sensor, wb_robot_get_basic_time_step());
      if (!joint.sensor && !joint.motor)
        throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);

      // Initialize the state
      joint.controlPosition = false;
      joint.controlVelocity = false;
      joint.controlEffort = false;
      joint.positionCommand = NAN;
      joint.velocityCommand = NAN;
      joint.effortCommand = NAN;
      joint.position = NAN;
      joint.velocity = NAN;
      joint.acceleration = NAN;

      // Check if state interfaces have initial positions
      for (hardware_interface::InterfaceInfo stateInterface : component.state_interfaces) {
        if (stateInterface.name == "position" && !stateInterface.initial_value.empty()) {
          joint.position = std::stod(stateInterface.initial_value);
          wb_motor_set_position(joint.motor, std::stod(stateInterface.initial_value));
        }
      }

      // Configure the command interface
      for (hardware_interface::InterfaceInfo commandInterface : component.command_interfaces) {
        if (commandInterface.name == "position")
          joint.controlPosition = true;
        else if (commandInterface.name == "velocity")
          joint.controlVelocity = true;
        else if (commandInterface.name == "effort")
          joint.controlEffort = true;
        else
          throw std::runtime_error("Invalid hardware info name `" + commandInterface.name + "`");
      }
      if (joint.motor && joint.controlVelocity && !joint.controlPosition) {
        wb_motor_set_position(joint.motor, INFINITY);
        wb_motor_set_velocity(joint.motor, 0.0);
      }

      mJoints.push_back(joint);
    }

    registerSensors(info);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> Ros2ControlSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (Joint &joint : mJoints) {
      if (joint.sensor) {
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
        interfaces.emplace_back(
          hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_ACCELERATION, &(joint.acceleration)));
      }
    }

    for (Imu& imu: mImus) {
      for (auto& state_interface: imu.state_interfaces) {
        if (imu_interface_name_map.find(state_interface) == imu_interface_name_map.end()) {
          throw std::runtime_error("Invalid IMU state interface name `" + state_interface + "`");
        }
        interfaces.emplace_back(
          hardware_interface::StateInterface(imu.name, state_interface, &(imu.imu_sensor_data[imu_interface_name_map.at(state_interface)])));
      }
    }

    for (ForceTorqueSensor& ft_sensor: mForceTorqueSensors) {
      for (auto& state_interface: ft_sensor.state_interfaces) {
        if (ft_interface_name_map.find(state_interface) == ft_interface_name_map.end()) {
          throw std::runtime_error("Invalid ForceTorqueSensor state interface name `" + state_interface + "`");
        }
        interfaces.emplace_back(
          hardware_interface::StateInterface(ft_sensor.name, state_interface, &(ft_sensor.force_torque_sensor_data[ft_interface_name_map.at(state_interface)])));
      }
    }

    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Ros2ControlSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (Joint &joint : mJoints)
      if (joint.motor) {
        if (joint.controlPosition)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));
        if (joint.controlEffort)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));
        if (joint.controlVelocity)
          interfaces.emplace_back(
            hardware_interface::CommandInterface(joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));
      }
    return interfaces;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Ros2ControlSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type Ros2ControlSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    static double lastReadTime = 0;

    const double deltaTime = wb_robot_get_time() - lastReadTime;
    lastReadTime = wb_robot_get_time();

    for (Joint &joint : mJoints) {
      if (joint.sensor) {
        const double position = wb_position_sensor_get_value(joint.sensor);
        const double velocity = std::isnan(joint.position) ? NAN : (position - joint.position) / deltaTime;

        if (!std::isnan(joint.velocity))
          joint.acceleration = (velocity - joint.velocity) / deltaTime;
        joint.velocity = velocity;
        joint.position = position;
      }
    }

    // std::cout << "joint positions: " << std::endl;
    // for (auto& joint: mJoints) {
    //     std::cout << " - " << joint.name << ": " << joint.position << std::endl;
    // }

    for (auto& imu: mImus) {
      if (wb_inertial_unit_get_sampling_period(imu.inertial) == 0 ||
          wb_gyro_get_sampling_period(imu.gyro) == 0 ||
          wb_accelerometer_get_sampling_period(imu.accelerometer) == 0) {
        continue;
      }

      if (imu.inertial) {
        const double *imu_data = wb_inertial_unit_get_quaternion(imu.inertial);
        imu.imu_sensor_data[imu_interface_name_map.at("orientation.x")] = imu_data[0];
        imu.imu_sensor_data[imu_interface_name_map.at("orientation.y")] = imu_data[1];
        imu.imu_sensor_data[imu_interface_name_map.at("orientation.z")] = imu_data[2];
        imu.imu_sensor_data[imu_interface_name_map.at("orientation.w")] = imu_data[3];
      }

      if (imu.gyro) {
        const double *gyro_data = wb_gyro_get_values(imu.gyro);
        imu.imu_sensor_data[imu_interface_name_map.at("angular_velocity.x")] = gyro_data[0];
        imu.imu_sensor_data[imu_interface_name_map.at("angular_velocity.y")] = gyro_data[1];
        imu.imu_sensor_data[imu_interface_name_map.at("angular_velocity.z")] = gyro_data[2];
      }

      if (imu.accelerometer) {
        const double *accel_data = wb_accelerometer_get_values(imu.accelerometer);
        imu.imu_sensor_data[imu_interface_name_map.at("linear_acceleration.x")] = accel_data[0];
        imu.imu_sensor_data[imu_interface_name_map.at("linear_acceleration.y")] = accel_data[1];
        imu.imu_sensor_data[imu_interface_name_map.at("linear_acceleration.z")] = accel_data[2];
      }
    }

    // std::unordered_map<std::string, double> force_mag;
    for (auto& ft_sensor: mForceTorqueSensors) {
      if (wb_touch_sensor_get_sampling_period(ft_sensor.touch_sensor) == 0) {
        continue;
      }

      if (ft_sensor.touch_sensor) {
        const double *force_3d_data = wb_touch_sensor_get_values(ft_sensor.touch_sensor);
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("force.x")] = force_3d_data[0];
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("force.y")] = force_3d_data[1];
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("force.z")] = force_3d_data[2];
        // TODO (rdu): torque values are not available in the Webots API at the moment
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("torque.x")] = 0.0;
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("torque.y")] = 0.0;
        ft_sensor.force_torque_sensor_data[ft_interface_name_map.at("torque.z")] = 0.0;

        // double force_magnitude = sqrt(pow(force_3d_data[0], 2) + pow(force_3d_data[1], 2) + pow(force_3d_data[2], 2));
        // force_mag[ft_sensor.name] = force_magnitude;
      }
    }
    // std::cout << "Force: ";
    // for (auto& [name, force]: force_mag) {
    //   std::cout << name << ": " << force << " ";
    // }
    // std::cout << std::endl;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type Ros2ControlSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    for (Joint &joint : mJoints) {
      if (joint.motor) {
        if (joint.controlPosition && !std::isnan(joint.positionCommand))
          wb_motor_set_position(joint.motor, joint.positionCommand);
        if (joint.controlVelocity && !std::isnan(joint.velocityCommand)) {
          // In the position control mode the velocity cannot be negative.
          const double velocityCommand = joint.controlPosition ? abs(joint.velocityCommand) : joint.velocityCommand;
          wb_motor_set_velocity(joint.motor, velocityCommand);
        }
        if (joint.controlEffort && !std::isnan(joint.effortCommand))
          wb_motor_set_torque(joint.motor, joint.effortCommand);
      }
    }
    return hardware_interface::return_type::OK;
  }

  void Ros2ControlSystem::registerSensors(const hardware_interface::HardwareInfo & hardware_info) {
    // Collect sensor handles
    size_t n_sensors = hardware_info.sensors.size();
    std::vector<hardware_interface::ComponentInfo> sensor_components_;

    for (unsigned int j = 0; j < n_sensors; j++) {
      hardware_interface::ComponentInfo component = hardware_info.sensors[j];
      sensor_components_.push_back(component);
      std::cout << "------------------------------------------------" << std::endl;
      std::cout << "Found sensor: " << component.name << std::endl;

      if (component.parameters.find("type") == component.parameters.end()) {
        std::cerr << "Sensor type not specified for sensor " << component.name << std::endl;
        continue;
      } else {
        std::string sensor_type = component.parameters.at("type");
        if (sensor_type == imu_type_name) {
          Imu imu;

          imu.name = component.name;
          std::string inertial_name = imu.name + "/inertial";
          std::string gyro_name = imu.name + "/gyro";
          std::string accelerometer_name = imu.name + "/accelerometer";

          for (auto& state_interface: component.state_interfaces) {
            imu.state_interfaces.push_back(state_interface.name);
          }

          imu.inertial = wb_robot_get_device(inertial_name.c_str());
          if (imu.inertial == 0 || wb_device_get_node_type(imu.inertial) != WB_NODE_INERTIAL_UNIT) {
            throw std::runtime_error("Cannot find InertialUnit with name " + imu.name);
          }

          imu.gyro = wb_robot_get_device(gyro_name.c_str());
          if (imu.gyro == 0 || wb_device_get_node_type(imu.gyro) != WB_NODE_GYRO) {
            throw std::runtime_error("Cannot find Gyro with name " + imu.name);
          }

          imu.accelerometer = wb_robot_get_device(accelerometer_name.c_str());
          if (imu.accelerometer == 0 || wb_device_get_node_type(imu.accelerometer) != WB_NODE_ACCELEROMETER) {
            throw std::runtime_error("Cannot find Accelerometer with name " + imu.name);
          }

          int sampling_period = wb_robot_get_basic_time_step();
          if (component.parameters.find("update_rate") != component.parameters.end()) {
            int update_rate = std::stoi(component.parameters.at("update_rate"));
            sampling_period = static_cast<int>(1000.0f / update_rate);
          }

          wb_inertial_unit_enable(imu.inertial, sampling_period);
          wb_gyro_enable(imu.gyro, sampling_period);
          wb_accelerometer_enable(imu.accelerometer, sampling_period);

          mImus.push_back(imu);

          std::cout << "IMU sensor " << imu.name << " registered successfully" << std::endl;
        } else if (sensor_type == ft_sensor_type_name) {
          ForceTorqueSensor ft_sensor;

          ft_sensor.name = component.name;

          for (auto& state_interface: component.state_interfaces) {
              ft_sensor.state_interfaces.push_back(state_interface.name);
          }

          ft_sensor.touch_sensor = wb_robot_get_device(ft_sensor.name.c_str());
          if (ft_sensor.touch_sensor == 0 || wb_device_get_node_type(ft_sensor.touch_sensor) != WB_NODE_TOUCH_SENSOR
            || wb_touch_sensor_get_type(ft_sensor.touch_sensor) != WB_TOUCH_SENSOR_FORCE3D) {
              throw std::runtime_error("Cannot find ForceTorqueSensor of type force-3d with name " + ft_sensor.name);
          }

          int sampling_period = wb_robot_get_basic_time_step();
          if (component.parameters.find("update_rate") != component.parameters.end()) {
            int update_rate = std::stoi(component.parameters.at("update_rate"));
            sampling_period = static_cast<int>(1000.0f / update_rate);
          }

          wb_touch_sensor_enable(ft_sensor.touch_sensor, sampling_period);

          mForceTorqueSensors.push_back(ft_sensor);

          std::cout << "ForceTorqueSensor " << ft_sensor.name << " registered successfully" << std::endl;
        } else {
          std::cerr << "Unknown sensor type: " << sensor_type << std::endl;
        }
      }
    }
  }
}  // namespace webots_ros2_control

PLUGINLIB_EXPORT_CLASS(webots_ros2_control::Ros2ControlSystem, webots_ros2_control::Ros2ControlSystemInterface)
