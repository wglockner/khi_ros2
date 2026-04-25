// Copyright 2025 Kawasaki Heavy Industries, Ltd.
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

#include "khi_hardware/khi_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "khi_hardware/khi_krnx_driver.hpp"
#include "khi_hardware/khi_mock_driver.hpp"
#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_publisher.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/khi_service.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiHardwareInterface::~KhiHardwareInterface() { driver_->error(); }

hardware_interface::CallbackReturn KhiHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_init");

  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = params.hardware_info;
  is_cleaning_up_ = false;
  is_deactivating_ = false;
  is_handling_error_ = false;
  is_shutdowning_ = false;
  is_active_ = false;
  write_enabled_ = false;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "Joint '%s' has %d command interface. size error.",
        joint.name.c_str(), (int)joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "Joint '%s' command interface type error.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "Joint '%s' has %d state interface. size error.",
        joint.name.c_str(), (int)joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("khi_hardware"),
        "Joint '%s' state interface type error.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  create_khi_robot_driver();

  auto result = driver_->initialize();
  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  service_ = std::make_shared<KhiService>();
  publisher_ = std::make_shared<KhiPublisher>();
  service_->init(*driver_);
  publisher_->init(*driver_);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "initialize success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_configure");

  auto result = driver_->configure();
  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  service_->start(*driver_);
  publisher_->start(*driver_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KhiHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & arm : driver_->get_robot().arms)
  {
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      state_interfaces.emplace_back(
        arm.joint_names[jt], hardware_interface::HW_IF_POSITION, &arm.state_positions[jt]);
      state_interfaces.emplace_back(
        arm.joint_names[jt], hardware_interface::HW_IF_VELOCITY, &arm.state_velocities[jt]);
      state_interfaces.emplace_back(
        arm.joint_names[jt], hardware_interface::HW_IF_EFFORT, &arm.state_efforts[jt]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KhiHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & arm : driver_->get_robot().arms)
  {
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      command_interfaces.emplace_back(
        arm.joint_names[jt], hardware_interface::HW_IF_POSITION, &arm.command_positions[jt]);
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto result = driver_->activate();

  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  is_active_ = true;
  write_enabled_ = true;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  is_deactivating_ = true;
  write_enabled_ = false;

  auto result = driver_->deactivate();

  is_active_ = false;
  is_deactivating_ = false;

  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KhiHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (is_cleaning_up_ || is_shutdowning_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!driver_->is_communicating())
  {
    if (is_handling_error_)
    {
      return hardware_interface::return_type::OK;
    }

    static rclcpp::Time last_error_time(0, 0, RCL_ROS_TIME);
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - last_error_time).seconds() > 1.0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Communication with the robot controller has been lost.");
      last_error_time = now;
    }

    return hardware_interface::return_type::ERROR;
  }

  static int health_monitor_counter = 0;
  if (++health_monitor_counter >= 50)
  {
    driver_->monitor_robot_health();
    health_monitor_counter = 0;
  }

  if (!driver_->read())
  {
    static rclcpp::Time last_read_error_time(0, 0, RCL_ROS_TIME);
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - last_read_error_time).seconds() > 1.0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "read err");
      last_read_error_time = now;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KhiHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (is_cleaning_up_ || is_deactivating_ || is_shutdowning_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!is_active_ || !write_enabled_)
  {
    return hardware_interface::return_type::OK;
  }

  if (!driver_->is_communicating())
  {
    static rclcpp::Time last_error_time(0, 0, RCL_ROS_TIME);
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - last_error_time).seconds() > 1.0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Communication with the robot controller has been lost.");
      last_error_time = now;
    }

    return hardware_interface::return_type::ERROR;
  }

  if (!driver_->is_writable())
  {
    static rclcpp::Time last_deactivate_log_time(0, 0, RCL_ROS_TIME);
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - last_deactivate_log_time).seconds() > 1.0)
    {
      RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "deactivate");
      last_deactivate_log_time = now;
    }

    return hardware_interface::return_type::DEACTIVATE;
  }

  if (!driver_->write())
  {
    static rclcpp::Time last_write_error_time(0, 0, RCL_ROS_TIME);
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - last_write_error_time).seconds() > 1.0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "write err");
      last_write_error_time = now;
    }

    return hardware_interface::return_type::DEACTIVATE;
  }

  return hardware_interface::return_type::OK;
}

// Remaining methods unchanged...
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(khi_hardware::KhiHardwareInterface, hardware_interface::SystemInterface)