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

#include "hardware_interface/introspection.hpp"

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
KhiHardwareInterface::~KhiHardwareInterface()
{
  join_transition_worker();
  // driver_ is null when on_init failed before create_khi_robot_driver().
  if (driver_)
  {
    driver_->error();
  }
}

void KhiHardwareInterface::start_transition_worker(
  const char * what, KhiResultCode (KhiDriver::*fn)())
{
  join_transition_worker();
  transition_what_ = what;
  transition_in_flight_.store(true);
  transition_worker_ = std::thread([this, fn]() {
    const auto result = (*driver_.*fn)();
    transition_result_ = result;
    if (result != KhiResultCode::SUCCESS)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "%s sequence FAILED in the background: the controller may still hold motor power or the "
        "RTC program. Check the pendant before the next activation.", transition_what_.c_str());
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("khi_hardware"), "%s sequence finished (background)",
        transition_what_.c_str());
    }
    is_deactivating_ = false;
    is_handling_error_ = false;
    transition_in_flight_.store(false);
  });
}

void KhiHardwareInterface::join_transition_worker()
{
  if (transition_worker_.joinable())
  {
    if (transition_in_flight_.load())
    {
      RCLCPP_INFO(
        rclcpp::get_logger("khi_hardware"), "waiting for the background %s sequence to finish",
        transition_what_.c_str());
    }
    transition_worker_.join();
  }
}

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

  if (!create_khi_robot_driver())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto result = driver_->initialize();
  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  service_ = std::make_shared<KhiService>();
  publisher_ = std::make_shared<KhiPublisher>();
  service_->init(*driver_);
  publisher_->init(*driver_);
  register_introspection();

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "initialize success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_configure");
  join_transition_worker();

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
  // A deactivate / error sequence still running in the background finishes first: the
  // controller is never held and powered on from two threads.
  join_transition_worker();
  if (transition_result_ != KhiResultCode::SUCCESS)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("khi_hardware"),
      "the previous %s sequence had failed; activating anyway (activate re-holds and powers on)",
      transition_what_.c_str());
    transition_result_ = KhiResultCode::SUCCESS;
  }
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
  is_active_ = false;

  // Non-blocking: the KRNX stop sequence runs on the worker (see the header); read() and
  // write() skip while is_deactivating_ is set, the state interfaces keep their last value
  // for that second and a half. The lifecycle reports INACTIVE at once; a failure of the
  // sequence is logged by the worker and re-checked at the next activation.
  start_transition_worker("deactivate", &KhiDriver::deactivate);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type KhiHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (is_cleaning_up_ || is_shutdowning_ || is_deactivating_ || is_handling_error_)
  {
    // A background stop sequence owns the controller's AUX/AS channel; no KRNX reads
    // from the control thread until it is done.
    return hardware_interface::return_type::OK;
  }

  if (!driver_->is_communicating())
  {
    if (is_handling_error_)
    {
      return hardware_interface::return_type::OK;
    }

    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - read_comm_loss_log_time_).seconds() > 1.0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Communication with the robot controller has been lost.");
      read_comm_loss_log_time_ = now;
    }

    return hardware_interface::return_type::ERROR;
  }

  if (++health_monitor_counter_ >= 50)
  {
    driver_->monitor_robot_health();
    health_monitor_counter_ = 0;
  }

  if (!driver_->read())
  {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - read_error_log_time_).seconds() > 1.0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "read err");
      read_error_log_time_ = now;
    }

    // A single failed cycle keeps the last state and moves on, but a sustained
    // failure means the state interfaces are silently stale — escalate.
    if (++consecutive_read_failures_ >= READ_FAILURE_ESCALATION_COUNT)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "%d consecutive read failures; state interfaces are stale. Reporting ERROR.",
        consecutive_read_failures_);
      return hardware_interface::return_type::ERROR;
    }
  }
  else
  {
    consecutive_read_failures_ = 0;
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
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - write_comm_loss_log_time_).seconds() > 1.0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("khi_hardware"),
        "Communication with the robot controller has been lost.");
      write_comm_loss_log_time_ = now;
    }

    return hardware_interface::return_type::ERROR;
  }

  if (!driver_->is_writable())
  {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - write_deactivate_log_time_).seconds() > 1.0)
    {
      RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "deactivate");
      write_deactivate_log_time_ = now;
    }

    return hardware_interface::return_type::DEACTIVATE;
  }

  if (!driver_->write())
  {
    auto now = rclcpp::Clock(RCL_ROS_TIME).now();

    if ((now - write_error_log_time_).seconds() > 1.0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "write err");
      write_error_log_time_ = now;
    }

    return hardware_interface::return_type::DEACTIVATE;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_cleanup");
  join_transition_worker();

  is_cleaning_up_ = true;
  is_active_ = false;
  write_enabled_ = false;
  if (service_)
  {
    service_->stop();
  }
  if (publisher_)
  {
    publisher_->stop();
  }

  auto result = driver_->cleanup();
  is_cleaning_up_ = false;
  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "Cleanup successful");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_shutdown");
  join_transition_worker();

  is_shutdowning_ = true;
  is_active_ = false;
  write_enabled_ = false;
  if (service_)
  {
    service_->stop();
    service_.reset();
  }
  if (publisher_)
  {
    publisher_->stop();
    publisher_.reset();
  }

  auto result = driver_->shutdown();
  is_shutdowning_ = false;
  if (result == KhiResultCode::FAILURE) return hardware_interface::CallbackReturn::FAILURE;
  if (result == KhiResultCode::ERROR) return hardware_interface::CallbackReturn::ERROR;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn KhiHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "on_error");

  is_handling_error_ = true;
  is_active_ = false;
  write_enabled_ = false;
  if (service_)
  {
    service_->stop();
  }
  if (publisher_)
  {
    publisher_->stop();
  }

  // Non-blocking like on_deactivate: with the link gone every KRNX call in the error
  // sequence runs into its timeout, which used to hold the whole control loop.
  start_transition_worker("error", &KhiDriver::error);
  return hardware_interface::CallbackReturn::SUCCESS;
}

int KhiHardwareInterface::get_arm_no(const hardware_interface::ComponentInfo & joint) const
{
  int arm_no = 0;
  if (joint.parameters.at("arm").find("arm2") != std::string::npos)
  {
    arm_no = 1;
  }
  return arm_no;
}

KhiRobotArmData KhiHardwareInterface::get_arm_info(const int target_arm_no) const
{
  KhiRobotArmData arm;

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (get_arm_no(joint) != target_arm_no)
    {
      continue;
    }

    arm.joint_types.push_back(joint.parameters.at("type"));
    arm.joint_names.push_back(joint.name);
    // URDF joint velocity limit = the controller's RTC step limit per cycle (the mock
    // driver refuses steps past it like the real controller does). Absent: unlimited.
    double max_velocity = 0.0;
    const auto lim = info_.limits.find(joint.name);
    if (lim != info_.limits.end() && lim->second.has_velocity_limits)
    {
      max_velocity = lim->second.max_velocity;
    }
    arm.max_velocities.push_back(max_velocity);
    for (const auto & command_interface : joint.command_interfaces)
    {
      if (command_interface.name == hardware_interface::HW_IF_POSITION)
      {
        arm.max_positions.push_back(std::stod(command_interface.max));
        arm.min_positions.push_back(std::stod(command_interface.min));
      }
      arm.control_modes.push_back(command_interface.name);
    }
  }

  arm.joint_num = static_cast<int>(arm.joint_names.size());
  arm.command_positions.resize(arm.joint_num, 0);
  arm.old_command_positions.resize(arm.joint_num, 0);
  arm.state_positions.resize(arm.joint_num, 0);
  arm.state_velocities.resize(arm.joint_num, 0);
  arm.state_efforts.resize(arm.joint_num, 0);

  return arm;
}

bool KhiHardwareInterface::create_khi_robot_driver()
try
{
  int max_arm_no = 0;
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    int arm_no = get_arm_no(joint);
    if (arm_no > max_arm_no)
    {
      max_arm_no = arm_no;
    }
  }

  std::vector<KhiRobotArmData> arms;
  for (int arm_no = 0; arm_no <= max_arm_no; arm_no++)
  {
    arms.push_back(get_arm_info(arm_no));
  }

  const int update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  if (update_rate <= 0 || update_rate > 1000 || 1000 % update_rate != 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("khi_hardware"),
      "update_rate parameter is %d Hz; it must be a positive divisor of 1000 (e.g. 100, 125, "
      "250, 500). A non-divisor rate silently truncates the RTC cycle time and desynchronizes "
      "the controller from ros2_control.",
      update_rate);
    return false;
  }

  KhiRobot robot;
  robot.controller_no = std::stoi(info_.hardware_parameters.at("controller_no"));
  robot.name = info_.hardware_parameters.at("robot_name");
  robot.ip_address = info_.hardware_parameters.at("robot_ip");
  robot.period = 1000 / update_rate;
  robot.arms = arms;
  // Optional (find, not at): URDFs written before the extended KRNX services stay valid.
  const auto undocumented_api = info_.hardware_parameters.find("enable_undocumented_krnx_api");
  robot.enable_undocumented_krnx_api =
    undocumented_api != info_.hardware_parameters.end() &&
    (undocumented_api->second == "True" || undocumented_api->second == "true");
  // Simulation tracking lag (KhiMockDriver). Optional; 0 = measured equals commanded.
  const auto sim_transition = info_.hardware_parameters.find("sim_transition_s");
  robot.sim_transition_s =
    (sim_transition != info_.hardware_parameters.end()) ? std::stod(sim_transition->second) : 0.0;
  const auto sim_lag = info_.hardware_parameters.find("sim_lag_s");
  robot.sim_lag_s = (sim_lag != info_.hardware_parameters.end()) ? std::stod(sim_lag->second) : 0.0;
  if (robot.sim_lag_s < 0.0 || robot.sim_lag_s > 2.0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("khi_hardware"),
      "sim_lag_s is %.3f s; it must be within 0..2 s (the CX110L measured 0.08..0.09 s).",
      robot.sim_lag_s);
    return false;
  }

  KhiPeriodicDataConfig config = {};
  config.is_actual_current_enabled =
    (info_.hardware_parameters.at("actual_current") == "True") ||
    (info_.hardware_parameters.at("actual_current") == "true");
  config.is_actual_encorder_enabled =
    (info_.hardware_parameters.at("actual_encorder") == "True") ||
    (info_.hardware_parameters.at("actual_encorder") == "true");
  config.is_command_current_enabled =
    (info_.hardware_parameters.at("command_current") == "True") ||
    (info_.hardware_parameters.at("command_current") == "true");
  config.is_command_encorder_enabled =
    (info_.hardware_parameters.at("command_encorder") == "True") ||
    (info_.hardware_parameters.at("command_encorder") == "true");
  config.is_tcp_info_enabled =
    (info_.hardware_parameters.at("tcp_info") == "True") ||
    (info_.hardware_parameters.at("tcp_info") == "true");
  config.is_external_signal_enabled =
    (info_.hardware_parameters.at("external_signal") == "True") ||
    (info_.hardware_parameters.at("external_signal") == "true");
  config.is_internal_signal_enabled =
    (info_.hardware_parameters.at("internal_signal") == "True") ||
    (info_.hardware_parameters.at("internal_signal") == "true");
  config.is_ft_sensor_enabled =
    (info_.hardware_parameters.at("ft_sensor") == "True") ||
    (info_.hardware_parameters.at("ft_sensor") == "true");

  const auto is_simulation =
    (info_.hardware_parameters.at("simulation") == "True") ||
    (info_.hardware_parameters.at("simulation") == "true");

  if (is_simulation)
  {
    driver_ = std::make_shared<KhiMockDriver>(robot, config);
    RCLCPP_INFO(
      rclcpp::get_logger("khi_hardware"),
      "KHI Robot Hardware Interface in simulation mode (tracking lag %.3f s, RTC step limits "
      "from the URDF velocity limits)", robot.sim_lag_s);
  }
  else
  {
    driver_ = std::make_shared<KhiKrnxDriver>(robot, config);
  }

  return true;
}
catch (const std::exception & e)
{
  // .at()/stoi/stod on hardware or joint parameters: a missing or non-numeric
  // entry lands here instead of throwing through controller_manager.
  RCLCPP_FATAL(
    rclcpp::get_logger("khi_hardware"),
    "Invalid or missing hardware configuration in the URDF <ros2_control> block: %s. Check "
    "the hardware parameters (robot_name, robot_ip, controller_no, update_rate, simulation, "
    "the periodic-data flags) and each joint's 'arm'/'type' params and position min/max.",
    e.what());
  return false;
}

void KhiHardwareInterface::register_introspection()
{
  // Published by the controller_manager at its update rate on
  // ~/introspection_data/{names,values} as "<component>.khi.<field>". The registry
  // exists only under a controller_manager (REGISTER_ENTITY is a no-op without it).
  KhiDriverHealth & h = driver_->health();
  try
  {
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.rtc_backlog", &h.rtc_backlog);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.rtc_refusals", &h.rtc_refusals);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.rtc_seq", &h.rtc_seq);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.write_errors", &h.write_errors);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.read_failures", &h.read_failures);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.feedback_frozen", &h.feedback_frozen);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.read_us", &h.read_us);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.write_us", &h.write_us);
    REGISTER_ROS2_CONTROL_INTROSPECTION("khi.sim_step_refusals", &h.sim_step_refusals);
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("khi_hardware"), "introspection registration failed: %s", e.what());
  }
}

}  // namespace khi_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(khi_hardware::KhiHardwareInterface, hardware_interface::SystemInterface)