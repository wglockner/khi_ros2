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

#ifndef KHI_HARDWARE__KHI_ROBOT_HPP_
#define KHI_HARDWARE__KHI_ROBOT_HPP_

#include <string>
#include <vector>

namespace khi_hardware
{
struct KhiRobotArmData
{
  int joint_num{};
  std::string name;
  std::vector<std::string> joint_names{};
  std::vector<std::string> joint_types{};
  std::vector<double> min_positions{};
  std::vector<double> max_positions{};
  std::vector<double> command_positions{};
  std::vector<double> old_command_positions{};
  std::vector<double> state_positions{};
  std::vector<double> state_velocities{};
  std::vector<double> state_efforts{};
  std::vector<std::string> control_modes{};
  // Controller RTC step limit per joint as a velocity [rad/s or m/s], from the URDF joint
  // velocity limit (the Kawasaki refuses a compensation step larger than this x the cycle;
  // 2026-10-27). 0 = unknown / unlimited. Used by the mock driver's step refusal only; the
  // real controller enforces its own.
  std::vector<double> max_velocities{};
};

struct KhiFTSensor
{
  double force_x;
  double force_y;
  double force_z;
  double torque_x;
  double torque_y;
  double torque_z;
  double counter_to_n_ratio;        // force [N] = counter_value * counter_to_n_ratio
  double counter_to_nm_ratio;       // torque [N] = counter_value * counter_to_nm_ratio
  bool enable_n_nm_output = false;  // true: output N or Nm, false: output counter value
};

struct KhiRobot
{
  std::string name;
  std::string ip_address;
  double period;
  int controller_no;
  std::vector<KhiRobotArmData> arms;
  KhiFTSensor ft_sensor;
  // Opt-in for the extended services that wrap KRNX functions the KRNX API manual does not
  // document AND that change controller state. Optional hardware parameter, default false.
  bool enable_undocumented_krnx_api = false;
  // Simulation only (KhiMockDriver): first-order tracking lag of the measured position
  // behind the command [s]. Measured on the CX110L 2026-10-22: 80..94 ms. 0 = the old
  // "measured equals commanded" mock. Hardware parameter `sim_lag_s`.
  double sim_lag_s = 0.0;
  // Simulation only: how long the mock's deactivate() / error() take [s] (the real KRNX
  // hold / kill / power-off sequence takes about 1.6 s). Hardware parameter `sim_transition_s`.
  double sim_transition_s = 0.0;
};
}  // namespace khi_hardware
#endif  // KHI_HARDWARE__KHI_ROBOT_HPP_
