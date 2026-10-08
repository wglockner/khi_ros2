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

#include "khi_hardware/khi_service.hpp"

#include <string>

#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
KhiService::~KhiService()
{
  stop();
  exit_.store(true);
  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }
}

void KhiService::init(const KhiDriver & driver)
{
  std::string node_namespace = "/khi_controller" + std::to_string(driver.get_robot().controller_no);
  node_ = rclcpp::Node::make_shared("khi_service", node_namespace);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  exit_.store(false);
  spin_thread_ = std::thread(
    [this]()
    {
      while (!exit_.load())
      {
        executor_->spin_once(std::chrono::milliseconds(100));
      }
    });
}

void KhiService::start(KhiDriver & driver)
{
  // A lifecycle transition (activate / deactivate) owns the KRNX AUX/AS channel while it runs (KhiDriver::is_transitioning()).
  static const std::string kTransitionRefusal =
    "refused: hardware lifecycle transition in progress (activate: motor power-on / RTC start; deactivate: hold, kill, motor power-off); retry once the "
    "component is active.";

  auto get_signal = [&driver](
                      const khi_msgs::srv::GetSignal::Request::SharedPtr & req,
                      const khi_msgs::srv::GetSignal::Response::SharedPtr & resp)
  {
    if (driver.is_transitioning())
    {
      resp->success = false;
      return;
    }
    driver.get_signal_srv_cb(req, resp);
  };
  get_signal_service_ = node_->create_service<khi_msgs::srv::GetSignal>("~/get_signal", get_signal);

  auto set_signal = [&driver](
                      const khi_msgs::srv::SetSignal::Request::SharedPtr & req,
                      const khi_msgs::srv::SetSignal::Response::SharedPtr & resp)
  {
    if (driver.is_transitioning())
    {
      resp->success = false;
      resp->error_msg = kTransitionRefusal;
      return;
    }
    driver.set_signal_srv_cb(req, resp);
  };
  set_signal_service_ = node_->create_service<khi_msgs::srv::SetSignal>("~/set_signal", set_signal);

  auto exec_khi_command = [&driver](
                            const khi_msgs::srv::ExecKhiCommand::Request::SharedPtr & req,
                            const khi_msgs::srv::ExecKhiCommand::Response::SharedPtr & resp)
  {
    if (driver.is_transitioning())
    {
      resp->success = false;
      resp->error_msg = kTransitionRefusal;
      return;
    }
    driver.exec_khi_command_srv_cb(req, resp);
  };
  khi_cmd_service_ =
    node_->create_service<khi_msgs::srv::ExecKhiCommand>("~/exec_khi_command", exec_khi_command);

  auto reset_error = [&driver](
                       const khi_msgs::srv::ResetError::Request::SharedPtr & req,
                       const khi_msgs::srv::ResetError::Response::SharedPtr & resp)
  {
    if (driver.is_transitioning())
    {
      resp->success = false;
      resp->error_msgs = {kTransitionRefusal};
      return;
    }
    // Reset every arm, not just arm 0 (duAro has two).
    bool all_ok = true;
    const int arm_count = static_cast<int>(driver.get_robot().arms.size());
    for (int arm_no = 0; arm_no < arm_count; arm_no++)
    {
      driver.reset_error_srv_cb(req, resp, arm_no);
      all_ok = all_ok && resp->success;
    }
    resp->success = all_ok;
  };
  reset_error_service_ =
    node_->create_service<khi_msgs::srv::ResetError>("~/reset_error", reset_error);

  auto set_timeout = [&driver](
                       const khi_msgs::srv::SetTimeout::Request::SharedPtr & req,
                       const khi_msgs::srv::SetTimeout::Response::SharedPtr & resp)
  { driver.set_timeout_srv_cb(req, resp); };
  set_timeout_service_ =
    node_->create_service<khi_msgs::srv::SetTimeout>("~/set_timeout", set_timeout);

  auto set_soft_bias = [&driver](
                         const khi_msgs::srv::SetATISoftwareBias::Request::SharedPtr & req,
                         const khi_msgs::srv::SetATISoftwareBias::Response::SharedPtr & resp)
  { driver.set_ati_software_bias_srv_cb(req, resp); };
  set_ati_software_bias_service_ = node_->create_service<khi_msgs::srv::SetATISoftwareBias>(
    "~/set_ati_software_bias", set_soft_bias);

  auto change_ft_output_mode =
    [&driver](
      const khi_msgs::srv::ChangeFTOutputMode::Request::SharedPtr & req,
      const khi_msgs::srv::ChangeFTOutputMode::Response::SharedPtr & resp)
  { driver.change_ft_output_mode_srv_cb(req, resp); };
  change_ft_output_mode_service_ = node_->create_service<khi_msgs::srv::ChangeFTOutputMode>(
    "~/change_ft_output_mode", change_ft_output_mode);

  driver.create_extended_services(*node_, extended_services_);

  RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "KhiService Start");
}

void KhiService::stop()
{
  get_signal_service_.reset();
  set_signal_service_.reset();
  khi_cmd_service_.reset();
  reset_error_service_.reset();
  set_timeout_service_.reset();
  set_ati_software_bias_service_.reset();
  change_ft_output_mode_service_.reset();
  extended_services_.clear();
}
}  // namespace khi_hardware
