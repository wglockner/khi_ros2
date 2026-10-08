// Copyright 2025 Kawasaki Heavy Industries, Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KHI_HARDWARE__KHI_DRIVER_HPP_
#define KHI_HARDWARE__KHI_DRIVER_HPP_

#include <math.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "khi_hardware/khi_periodic_data_config.hpp"
#include "khi_hardware/khi_result_code.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/msg/error_info.hpp"
#include "khi_msgs/srv/change_ft_output_mode.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "khi_msgs/srv/get_signal.hpp"
#include "khi_msgs/srv/reset_error.hpp"
#include "khi_msgs/srv/set_ati_software_bias.hpp"
#include "khi_msgs/srv/set_signal.hpp"
#include "khi_msgs/srv/set_timeout.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
class KhiDriver
{
public:
  explicit KhiDriver(KhiRobot robot, KhiPeriodicDataConfig periodic_data)
  : robot_(std::move(robot)), periodic_data_config_(periodic_data)
  {
  }
  virtual ~KhiDriver() = 0;
  virtual KhiResultCode initialize() = 0;
  virtual KhiResultCode configure() = 0;
  virtual KhiResultCode activate() = 0;
  virtual KhiResultCode deactivate() = 0;
  virtual KhiResultCode cleanup() = 0;
  virtual KhiResultCode shutdown() = 0;
  virtual KhiResultCode error() = 0;
  virtual bool read() = 0;
  virtual bool write() = 0;
  virtual bool is_writable() = 0;
  virtual void monitor_robot_health() = 0;

  virtual void get_signal_srv_cb(
    const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::GetSignal::Response::SharedPtr & resp) const = 0;
  virtual void set_signal_srv_cb(
    const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
    const khi_msgs::srv::SetSignal::Response::SharedPtr & resp) const = 0;
  virtual void exec_khi_command_srv_cb(
    const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
    const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp) const = 0;
  virtual void reset_error_srv_cb(
    const khi_msgs::srv::ResetError::Request::SharedPtr & req,
    const khi_msgs::srv::ResetError::Response::SharedPtr & resp, int arm_no) const = 0;
  virtual void set_timeout_srv_cb(
    const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
    const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp) const = 0;
  virtual void set_ati_software_bias_srv_cb(
    const khi_msgs::srv::SetATISoftwareBias::Request::SharedPtr & req,
    const khi_msgs::srv::SetATISoftwareBias::Response::SharedPtr & resp) const = 0;
  virtual void change_ft_output_mode_srv_cb(
    const khi_msgs::srv::ChangeFTOutputMode::Request::SharedPtr & req,
    const khi_msgs::srv::ChangeFTOutputMode::Response::SharedPtr & resp) = 0;
  // Extended driver-specific services (the KRNX API surface). Created on the KhiService node;
  // the returned handles are owned by KhiService and dropped in KhiService::stop().
  virtual void create_extended_services(
    rclcpp::Node & /*node*/, std::vector<rclcpp::ServiceBase::SharedPtr> & /*services*/)
  {
  }
  virtual bool is_error() const = 0;
  virtual bool get_error_info(
    std::vector<int> & error_codes, std::vector<std::string> & error_msgs) const = 0;
  virtual bool get_actual_current(std::vector<float> & actual_current) const = 0;
  virtual bool is_communicating() const = 0;

  /**
   * @brief True while activate() or deactivate() (also shutdown/error) runs its
   * KRNX sequence: power-on / RTC start, or hold / kill / power-off.
   *
   * The KRNX AUX/AS channel is reserved for the transition during that window:
   * the krnx_PowerOn/OffMotorWith*Wait wrappers are one monitor command followed
   * by a series of panel reads, and the library keeps that panel state in a global
   * buffer, so a publisher or service call landing between those reads can make
   * the power-on wrongly report "motor lamp never lit" (seen on the K2 cell,
   * 2026-09-19). Publishers skip their cycle and services refuse while this is set.
   */
  bool is_transitioning() const { return transitioning_.load(); }

  KhiRobot & get_robot() { return robot_; }

  const KhiRobot & get_robot() const { return robot_; }

protected:
  std::string convert_encoding(const char * input, const char * dst, const char * src) const;

  static constexpr int KHI_MAX_SIG_SIZE = 512;
  static constexpr int M2MM = 1000;
  static constexpr int ON = -1;
  static constexpr int OFF = 0;
  static constexpr int PRINT_THRESH = 1000;
  static constexpr int ERROR_RESET_TIME = 200;  // [ms]
  // Set by activate() / deactivate() / shutdown() / error() for their duration (see
  // is_transitioning()).
  std::atomic<bool> transitioning_{false};
  static constexpr double MSEC2SEC = 0.001;
  static constexpr double RAD2DEG = 180 / M_PI;
  static constexpr double MM2M = 0.001;
  const std::string jt_type_prismatic_ = "prismatic";

  KhiRobot robot_;
  KhiPeriodicDataConfig periodic_data_config_;
  // Guards robot_.ft_sensor configuration: change_ft_output_mode_srv_cb writes
  // it from the service thread while read() consumes it on the control thread.
  mutable std::mutex ft_config_mutex_;
};

}  // namespace khi_hardware

#endif  // KHI_HARDWARE__KHI_DRIVER_HPP_
