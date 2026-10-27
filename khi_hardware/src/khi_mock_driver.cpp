// Copyright 2025 Kawasaki Heavy Industries, Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "khi_hardware/khi_mock_driver.hpp"

#include <algorithm>
#include <cmath>

#include "khi_hardware/khi_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_msgs/srv/exec_khi_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{

// The mock models what the real CX110L does with a position stream (2026-10-27):
//
//  * Tracking lag. The measured position follows the command through a first-order
//    lag of `sim_lag_s` (KhiRobot::sim_lag_s, hardware parameter sim_lag_s). At constant
//    speed a first-order lag trails by exactly its time constant, and the single-track
//    records of 2026-10-22 put the arm 80..94 ms behind the controller timeline, so the
//    default 0.09 s reproduces the gate timing and the surface guard's view of the arm.
//    0 restores the old "measured equals commanded" mock.
//  * Step refusal. The controller refuses an RTC compensation point whose step from the
//    previous point exceeds its per-joint limit (velocity limit x cycle; the logged
//    spd_limit values match the URDF joint velocity limits) and HOLDS the previous
//    point: every later point then differs from it by more than the limit and is refused
//    too, so the arm freezes while commands stream (seen on the cell as "RTC comp
//    REFUSED"). The mock does the same with KhiRobotArmData::max_velocities, logs it
//    like the real driver and counts it in health().sim_step_refusals.
bool KhiMockDriver::read()
{
  const double dt = robot_.period * MSEC2SEC;
  const double lag = std::max(0.0, robot_.sim_lag_s);
  // Exact first-order response over one cycle: x += (u - x) * (1 - exp(-dt / tau)).
  const double alpha = (lag > 0.0 && dt > 0.0) ? (1.0 - std::exp(-dt / lag)) : 1.0;
  if (sim_target_.size() != robot_.arms.size())
  {
    sim_target_.resize(robot_.arms.size());
    for (size_t a = 0; a < robot_.arms.size(); a++)
    {
      sim_target_[a] = robot_.arms[a].command_positions;
    }
  }
  for (size_t a = 0; a < robot_.arms.size(); a++)
  {
    auto & arm = robot_.arms[a];
    for (int jt = 0; jt < static_cast<int>(arm.joint_names.size()); jt++)
    {
      const double prev = arm.state_positions[jt];
      const double next = prev + (sim_target_[a][jt] - prev) * alpha;
      arm.state_positions[jt] = next;
      arm.state_velocities[jt] = (dt > 0.0) ? (next - prev) / dt : 0.0;
      arm.state_efforts[jt] = 0.0;
    }
  }
  if ((sim_cnt_read_ - 1) % PRINT_THRESH == 0)
  {
    constexpr int msg_size = 512;
    char msg[msg_size] = {0};
    int offset = 0;
    for (const auto & arm : robot_.arms)
    {
      for (int jt = 0; jt < arm.joint_num; jt++)
      {
        int written = snprintf(msg + offset, msg_size - offset, "%.3lf\t", arm.state_positions[jt]);
        if ((written < 0) || (written >= msg_size - offset))
        {
          break;
        }
        offset += written;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "[SIM] read %s", msg);
  }
  sim_cnt_read_++;
  return true;
}

bool KhiMockDriver::write()
{
  const double dt = robot_.period * MSEC2SEC;
  if (sim_target_.size() != robot_.arms.size())
  {
    sim_target_.resize(robot_.arms.size());
    for (size_t a = 0; a < robot_.arms.size(); a++)
    {
      sim_target_[a] = robot_.arms[a].command_positions;
    }
  }
  for (size_t a = 0; a < robot_.arms.size(); a++)
  {
    const auto & arm = robot_.arms[a];
    for (int jt = 0; jt < arm.joint_num; jt++)
    {
      const double cmd = arm.command_positions[jt];
      const double limit_v = (jt < static_cast<int>(arm.max_velocities.size()))
                               ? arm.max_velocities[jt] : 0.0;
      const double step = std::abs(cmd - sim_target_[a][jt]);
      if (limit_v > 0.0 && dt > 0.0 && step > limit_v * dt)
      {
        // Refused: the accepted point stays where it was, exactly like the controller.
        health_.sim_step_refusals += 1.0;
        const auto now = rclcpp::Clock(RCL_ROS_TIME).now();
        if ((now - step_refuse_log_time_).seconds() > 1.0)
        {
          step_refuse_log_time_ = now;
          const bool prismatic = (arm.joint_types[jt] == jt_type_prismatic_);
          const double unit = prismatic ? 1.0 : RAD2DEG;
          RCLCPP_ERROR(
            rclcpp::get_logger("khi_hardware"),
            "[SIM] A commanded position exceeding the speed limit was sent [arm_no:%zu]. "
            "( JT%d cmd:%.4f%s old_cmd:%.4f%s cmd_spd:%.2f%s spd_limit:%.2f%s ) The point is "
            "REFUSED and the arm holds the previous point, as the controller does.",
            a + 1, jt + 1, cmd * unit, prismatic ? "[m]" : "[deg]",
            sim_target_[a][jt] * unit, prismatic ? "[m]" : "[deg]",
            step / dt * unit, prismatic ? "[m/s]" : "[deg/s]",
            limit_v * unit, prismatic ? "[m/s]" : "[deg/s]");
        }
        continue;
      }
      sim_target_[a][jt] = cmd;
    }
  }
  if ((sim_cnt_write_ - 1) % PRINT_THRESH == 0)
  {
    constexpr int msg_size = 512;
    char msg[msg_size] = {0};
    int offset = 0;
    for (const auto & arm : robot_.arms)
    {
      for (int jt = 0; jt < arm.joint_num; jt++)
      {
        int written =
          snprintf(msg + offset, msg_size - offset, "%.3lf\t", arm.command_positions[jt]);
        if ((written < 0) || (written >= msg_size - offset))
        {
          break;
        }
        offset += written;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("khi_hardware"), "[SIM] write %s", msg);
  }
  sim_cnt_write_++;
  return true;
}
}  // namespace khi_hardware
