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

// The KRNX API surface exposed as ROS 2 services (see docs/krnx_api_services.md).
//
// Ground rules, in order of precedence:
//  - The driver owns the controller connection and the RTC stream. KRNX functions that open or
//    close the connection, or that push data into the RT cyclic channel the control loop is
//    streaming on, are NOT wrapped here (krnx_Open, krnx_eth_*, krnx_RtcInit,
//    krnx_SetRtcCompData, krnx_SetRtcCompDataEx, krnx_SendRtCtlData, krnx_PrimeRtIoData,
//    krnx_PrimeCommandPosition). A service thread calling them would inject into, or tear
//    down, a live motion stream.
//  - Functions documented in the KRNX API Manual (90210-1416DEC, DEV 2.3.0) are always on.
//  - Undocumented functions that only read fixed-size data are always on; every output buffer
//    is oversized because their required sizes are not specified.
//  - Undocumented functions that change controller state refuse unless the hardware parameter
//    enable_undocumented_krnx_api is true.
//
// KRNX AUX/AS/RT APIs are thread-safe (manual chapter 11), so calling them from the KhiService
// thread beside the control loop is supported; they are not reentrant, and the single-threaded
// KhiService executor never re-enters them.

#include <algorithm>
#include <array>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "khi_hardware/khi_krnx_driver.hpp"
#include "khi_hardware/khi_robot.hpp"
#include "khi_hardware/krnx.h"
#include "khi_msgs/msg/krnx_result.hpp"
#include "khi_msgs/msg/mon_info.hpp"
#include "khi_msgs/msg/motion_info.hpp"
#include "khi_msgs/msg/panel_info.hpp"
#include "khi_msgs/msg/stepper_info.hpp"
#include "khi_msgs/srv/buffer_receive.hpp"
#include "khi_msgs/srv/buffer_send.hpp"
#include "khi_msgs/srv/buffer_status.hpp"
#include "khi_msgs/srv/delete_data.hpp"
#include "khi_msgs/srv/do_instruction.hpp"
#include "khi_msgs/srv/forward_kinematics.hpp"
#include "khi_msgs/srv/get_config.hpp"
#include "khi_msgs/srv/get_cycle_count.hpp"
#include "khi_msgs/srv/get_error_code.hpp"
#include "khi_msgs/srv/get_frame.hpp"
#include "khi_msgs/srv/get_io_info.hpp"
#include "khi_msgs/srv/get_joint_limits.hpp"
#include "khi_msgs/srv/get_motion_info.hpp"
#include "khi_msgs/srv/get_panel_info.hpp"
#include "khi_msgs/srv/get_program_info.hpp"
#include "khi_msgs/srv/get_rtc_state.hpp"
#include "khi_msgs/srv/get_signal_aux.hpp"
#include "khi_msgs/srv/inverse_kinematics.hpp"
#include "khi_msgs/srv/jacobian.hpp"
#include "khi_msgs/srv/list_data.hpp"
#include "khi_msgs/srv/load_data.hpp"
#include "khi_msgs/srv/motor_power.hpp"
#include "khi_msgs/srv/program_control.hpp"
#include "khi_msgs/srv/record_step.hpp"
#include "khi_msgs/srv/reset_signals.hpp"
#include "khi_msgs/srv/save_data.hpp"
#include "khi_msgs/srv/set_as_api_timeout.hpp"
#include "khi_msgs/srv/set_conveyor.hpp"
#include "khi_msgs/srv/set_frame.hpp"
#include "khi_msgs/srv/set_io.hpp"
#include "khi_msgs/srv/set_jt1_mode.hpp"
#include "khi_msgs/srv/set_monitor_speed.hpp"
#include "khi_msgs/srv/set_panel_info.hpp"
#include "khi_msgs/srv/set_rtc_comp_mask.hpp"
#include "khi_msgs/srv/set_rtc_error_flag.hpp"
#include "khi_msgs/srv/set_signal_aux.hpp"
#include "khi_msgs/srv/transform_math.hpp"
#include "rclcpp/rclcpp.hpp"

namespace khi_hardware
{
namespace
{
namespace srv = khi_msgs::srv;
using khi_msgs::msg::KrnxResult;

constexpr int XYZOAT_SIZE = 6;
constexpr int MATRIX_SIZE = 12;
constexpr int JACOBIAN_SIZE = 36;
// Output buffers of functions the manual does not size are padded to this many elements.
constexpr int UNSIZED_BUF = 64;
constexpr int LIST_BUF_SIZE = 64 * 1024;
constexpr int MOTION_SYNC_MAX = 63;  // libkrnx rejects data_num > 0x3f
constexpr int BUFFER_RECV_MAX = 4096;
constexpr int BUFFER_RECV_CAPACITY = 65536;
constexpr int PC_PROGRAM_MIN = 1;

template <typename SrvT>
void add_service(
  const KhiDriver & driver, rclcpp::Node & node,
  std::vector<rclcpp::ServiceBase::SharedPtr> & services, const std::string & name,
  std::function<void(const typename SrvT::Request &, typename SrvT::Response &)> callback)
{
  services.push_back(node.create_service<SrvT>(
    name, [&driver, name, callback](
            const std::shared_ptr<typename SrvT::Request> req,
            std::shared_ptr<typename SrvT::Response> resp)
    {
      // Every response embeds a KrnxResult. While activate() owns the AUX/AS channel a
      // call from here could corrupt its power-on lamp reads, so refuse instead.
      if (driver.is_transitioning())
      {
        resp->result.success = false;
        resp->result.error_msg =
          name + " refused: hardware lifecycle transition in progress (activate: motor power-on / RTC start; deactivate: hold, kill, motor power-off); "
                 "retry once the component is active.";
        return;
      }
      callback(*req, *resp);
    }));
}

template <typename OutT, typename InT>
std::vector<OutT> to_vector(const InT * data, const size_t size)
{
  return std::vector<OutT>(data, data + size);
}

void refuse(KrnxResult & result, const std::string & reason)
{
  result.success = false;
  result.error_msg = reason;
  RCLCPP_ERROR(rclcpp::get_logger("khi_hardware"), "%s", reason.c_str());
}

khi_msgs::msg::StepperInfo to_msg(const TKrnxStepperInfo & info)
{
  khi_msgs::msg::StepperInfo msg;
  msg.status = info.status;
  msg.exec_count = info.exec_count;
  msg.remain_count = info.remain_count;
  msg.program_name =
    std::string(info.program_name, strnlen(info.program_name, sizeof(info.program_name)));
  msg.priority = info.priority;
  msg.step_number = info.step_number;
  msg.step_name = std::string(info.step_name, strnlen(info.step_name, sizeof(info.step_name)));
  return msg;
}

khi_msgs::msg::MotionInfo to_msg(const TKrnxMotionInfo & info)
{
  khi_msgs::msg::MotionInfo msg;
  msg.ang = to_vector<float>(info.ang, KRNX_MAXAXES);
  msg.ang_ref = to_vector<float>(info.ang_ref, KRNX_MAXAXES);
  msg.vel = to_vector<float>(info.vel, KRNX_MAXAXES);
  msg.vel_ref = to_vector<int64_t>(info.vel_ref, KRNX_MAXAXES);
  msg.cur_ref = to_vector<float>(info.cur_ref, KRNX_MAXAXES);
  msg.enc = to_vector<int64_t>(info.enc, KRNX_MAXAXES);
  return msg;
}
}  // namespace

void KhiKrnxDriver::fill_krnx_result(
  KrnxResult & result, const int krnx_code, const int as_err_code) const
{
  result.krnx_code = krnx_code;
  result.as_error_code = as_err_code;
  // KRNX errors are all negative; a few calls return a non-negative count or state instead.
  result.success = (krnx_code >= KRNX_NOERROR) && (as_err_code == 0);

  if (krnx_code < KRNX_NOERROR)
  {
    std::stringstream ss;
    ss << std::hex << -krnx_code;
    result.krnx_err = "-0x" + ss.str();
  }

  if (as_err_code != 0)
  {
    int error_code = 0;
    char msg[KRNX_MSGSIZE] = {};
    const std::string cmd = "TYPE $ERROR(" + std::to_string(as_err_code) + ")";
    exec_monitor_command(robot_.controller_no, cmd.c_str(), msg, sizeof(msg), &error_code, true);
    result.error_msg = convert_to_utf8(msg);
  }
}

bool KhiKrnxDriver::is_arm_valid(KrnxResult & result, const int arm_no) const
{
  if (arm_no >= 0 && arm_no < static_cast<int>(robot_.arms.size()))
  {
    return true;
  }
  refuse(
    result, "arm_no " + std::to_string(arm_no) + " is out of range: controller " +
              std::to_string(robot_.controller_no) + " has arms 0.." +
              std::to_string(static_cast<int>(robot_.arms.size()) - 1) + " (0-based).");
  return false;
}

bool KhiKrnxDriver::is_undocumented_api_enabled(KrnxResult & result, const char * api) const
{
  if (robot_.enable_undocumented_krnx_api)
  {
    return true;
  }
  refuse(
    result, std::string(api) +
              " is not covered by the KRNX API manual and changes controller state, so it is "
              "disabled. To enable it add <param name=\"enable_undocumented_krnx_api\">true"
              "</param> to the <hardware> block of this robot's <ros2_control> URDF tag.");
  return false;
}

bool KhiKrnxDriver::get_current_joints(KrnxResult & result, const int arm_no, float * joint) const
{
  TKrnxCurMotionData motion = {};
  const int return_code = krnx_GetCurMotionData(robot_.controller_no, arm_no, &motion);
  if (return_code != KRNX_NOERROR)
  {
    fill_krnx_result(result, return_code, 0);
    result.error_msg = "krnx_GetCurMotionData failed while reading the current joint positions.";
    return false;
  }
  std::copy(motion.ang, motion.ang + KRNX_MAXAXES, joint);
  return true;
}

void KhiKrnxDriver::create_extended_services(
  rclcpp::Node & node, std::vector<rclcpp::ServiceBase::SharedPtr> & services)
{
  const int cont_no = robot_.controller_no;
  const auto logger = rclcpp::get_logger("khi_hardware");

  // Copies `joint` (or the current joints when empty) into an 18-axis buffer.
  auto resolve_joints = [this](
                          KrnxResult & result, const int arm_no, const std::vector<float> & joint,
                          float * out) -> bool
  {
    if (joint.empty())
    {
      return get_current_joints(result, arm_no, out);
    }
    if (joint.size() > KRNX_MAXAXES)
    {
      refuse(result, "joint has more than KRNX_MAXAXES (18) entries.");
      return false;
    }
    std::fill(out, out + KRNX_MAXAXES, 0.0F);
    std::copy(joint.begin(), joint.end(), out);
    return true;
  };

  /**************************** AUX-API: program control ****************************/

  add_service<srv::ProgramControl>(
    *this, node, services, "~/program_control",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      using Req = srv::ProgramControl::Request;
      const bool is_pc = req.target == Req::TARGET_PC;
      if (req.target != Req::TARGET_ROBOT && !is_pc)
      {
        refuse(resp.result, "target must be TARGET_ROBOT (0) or TARGET_PC (1).");
        return;
      }
      if (is_pc && (req.number < PC_PROGRAM_MIN || req.number > KRNX_MAX_PCPROG))
      {
        refuse(resp.result, "PC program number must be 1-3.");
        return;
      }
      if (!is_pc && !is_arm_valid(resp.result, req.number))
      {
        return;
      }

      RCLCPP_INFO(
        logger, "program_control: target:%d number:%d action:%d program:'%s'", req.target,
        req.number, req.action, req.program.c_str());

      int as_err = 0;
      int code = KRNX_NOERROR;
      const char * program = req.program.c_str();
      switch (req.action)
      {
        case Req::ACTION_EXECUTE:
          code = is_pc
                   ? krnx_PcExecute(cont_no, req.number, program, req.exec_num, req.step_num, &as_err)
                   : krnx_Execute(cont_no, req.number, program, req.exec_num, req.step_num, &as_err);
          break;
        case Req::ACTION_CONTINUE:
          code = is_pc ? krnx_PcContinue(cont_no, req.number, req.next_step ? 1 : 0, &as_err)
                       : krnx_Continue(cont_no, req.number, req.next_step ? 1 : 0, &as_err);
          break;
        case Req::ACTION_ABORT:
          code = is_pc ? krnx_PcAbort(cont_no, req.number, &as_err)
                       : krnx_Abort(cont_no, req.number, &as_err);
          break;
        case Req::ACTION_KILL:
          code = is_pc ? krnx_PcKill(cont_no, req.number, &as_err)
                       : krnx_Kill(cont_no, req.number, &as_err);
          break;
        case Req::ACTION_PRIME:
          if (is_pc)
          {
            refuse(resp.result, "ACTION_PRIME applies to TARGET_ROBOT only.");
            return;
          }
          code = krnx_Prime(
            cont_no, req.number, program, req.exec_num, req.step_num, req.create ? 1 : 0, &as_err);
          break;
        case Req::ACTION_HOLD:
          if (is_pc)
          {
            refuse(resp.result, "ACTION_HOLD applies to TARGET_ROBOT only (PC: ACTION_ABORT).");
            return;
          }
          code = krnx_Hold(cont_no, req.number, &as_err);
          break;
        case Req::ACTION_END:
          if (!is_pc)
          {
            refuse(resp.result, "ACTION_END applies to TARGET_PC only (robot: ACTION_ABORT).");
            return;
          }
          code = krnx_PcEnd(cont_no, req.number, &as_err);
          break;
        default:
          refuse(resp.result, "Unknown action " + std::to_string(req.action) + ".");
          return;
      }
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::MotorPower>(
    *this, node, services, "~/motor_power",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      RCLCPP_INFO(logger, "motor_power: arm_no:%d on:%d", req.arm_no, req.on);
      int as_err = 0;
      const int code = req.on ? krnx_PowerOnMotorWithOnWait(cont_no, req.arm_no, &as_err)
                              : krnx_PowerOffMotorWithOffWait(cont_no, req.arm_no, &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::SetMonitorSpeed>(
    *this, node, services, "~/set_monitor_speed",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      RCLCPP_INFO(logger, "set_monitor_speed: arm_no:%d speed:%.1f%%", req.arm_no, req.speed);
      int as_err = 0;
      const int code = krnx_SetMonSpeed(cont_no, req.arm_no, req.speed, &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::DoInstruction>(
    *this, node, services, "~/do_instruction",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      RCLCPP_INFO(
        logger, "do_instruction: arm_no:%d instruction:'%s'", req.arm_no, req.instruction.c_str());
      int as_err = 0;
      const int code = krnx_Do(cont_no, req.arm_no, req.instruction.c_str(), &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::RecordStep>(
    *this, node, services, "~/record_step",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      RCLCPP_INFO(
        logger, "record_step: program:'%s' step:%d insert:%d data:'%s'", req.program.c_str(),
        req.step_num, req.insert, req.step_data.c_str());
      int as_err = 0;
      const int code = krnx_RecOneStep(
        cont_no, req.program.c_str(), req.step_num, req.step_data.c_str(), req.insert ? 1 : 0,
        &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  /**************************** AUX-API: frames, signals, data ****************************/

  add_service<srv::GetFrame>(
    *this, node, services, "~/get_frame",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::GetFrame::Request;
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      if (req.frame != Req::FRAME_TOOL && req.frame != Req::FRAME_BASE)
      {
        refuse(resp.result, "frame must be FRAME_TOOL (0) or FRAME_BASE (1).");
        return;
      }
      int as_err = 0;
      float xyzoat[UNSIZED_BUF] = {};
      const int code = (req.frame == Req::FRAME_TOOL)
                         ? krnx_GetToolMatrix(cont_no, req.arm_no, xyzoat, &as_err)
                         : krnx_GetBaseMatrix(cont_no, req.arm_no, xyzoat, &as_err);
      fill_krnx_result(resp.result, code, as_err);
      std::copy(xyzoat, xyzoat + XYZOAT_SIZE, resp.xyzoat.begin());
    });

  add_service<srv::SetFrame>(
    *this, node, services, "~/set_frame",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      using Req = srv::SetFrame::Request;
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      if (req.frame != Req::FRAME_TOOL && req.frame != Req::FRAME_BASE)
      {
        refuse(resp.result, "frame must be FRAME_TOOL (0) or FRAME_BASE (1).");
        return;
      }
      RCLCPP_INFO(
        logger, "set_frame: arm_no:%d %s = [%.3f %.3f %.3f mm, %.5f %.5f %.5f rad]", req.arm_no,
        (req.frame == Req::FRAME_TOOL) ? "TOOL" : "BASE", req.xyzoat[0], req.xyzoat[1],
        req.xyzoat[2], req.xyzoat[3], req.xyzoat[4], req.xyzoat[5]);
      int as_err = 0;
      const int code = (req.frame == Req::FRAME_TOOL)
                         ? krnx_SetToolMatrix(cont_no, req.arm_no, req.xyzoat.data(), &as_err)
                         : krnx_SetBaseMatrix(cont_no, req.arm_no, req.xyzoat.data(), &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::GetSignalAux>(
    *this, node, services, "~/get_signal_aux",
    [this, cont_no](const auto & req, auto & resp)
    {
      int as_err = 0;
      int status = 0;
      const int code = krnx_GetSignal(cont_no, req.signal_number, &status, &as_err);
      fill_krnx_result(resp.result, code, as_err);
      resp.is_on = (status != 0);
    });

  add_service<srv::SetSignalAux>(
    *this, node, services, "~/set_signal_aux",
    [this, cont_no](const auto & req, auto & resp)
    {
      int as_err = 0;
      const int code = krnx_SetSignal(cont_no, req.signal_number, req.is_on ? ON : OFF, &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::ResetSignals>(
    *this, node, services, "~/reset_signals",
    [this, cont_no, logger](const auto & /*req*/, auto & resp)
    {
      RCLCPP_INFO(logger, "reset_signals: turning OFF all external output signals");
      int as_err = 0;
      const int code = krnx_Reset(cont_no, &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::DeleteData>(
    *this, node, services, "~/delete_data",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (req.element_name.empty())
      {
        refuse(resp.result, "element_name is empty. Use \"*\" to delete all of element_type.");
        return;
      }
      RCLCPP_INFO(
        logger, "delete_data: element:'%s' type:0x%X", req.element_name.c_str(), req.element_type);
      int as_err = 0;
      const int code = krnx_Delete(cont_no, req.element_name.c_str(), req.element_type, &as_err);
      fill_krnx_result(resp.result, code, as_err);
    });

  add_service<srv::ListData>(
    *this, node, services, "~/list_data",
    [this, cont_no](const auto & req, auto & resp)
    {
      int as_err = 0;
      std::vector<char> buffer(LIST_BUF_SIZE, '\0');
      const char * element = req.element_name.empty() ? nullptr : req.element_name.c_str();
      const int code = krnx_List(
        cont_no, element, req.element_type, buffer.data(), static_cast<int>(buffer.size()) - 1,
        &as_err);
      fill_krnx_result(resp.result, code, as_err);
      resp.listing = convert_to_utf8(buffer.data());
    });

  add_service<srv::SaveData>(
    *this, node, services, "~/save_data",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (req.filename.empty())
      {
        refuse(resp.result, "filename is empty.");
        return;
      }
      RCLCPP_INFO(
        logger, "save_data: file:'%s' program:'%s' option:0x%X", req.filename.c_str(),
        req.program_name.c_str(), req.option);
      const int code =
        krnx_Save(cont_no, req.filename.c_str(), req.program_name.c_str(), req.option);
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::LoadData>(
    *this, node, services, "~/load_data",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (req.filename.empty())
      {
        refuse(resp.result, "filename is empty.");
        return;
      }
      RCLCPP_INFO(logger, "load_data: file:'%s'", req.filename.c_str());
      const int code = krnx_LoadEx(cont_no, req.filename.c_str(), nullptr, nullptr);
      fill_krnx_result(resp.result, code, 0);
    });

  /**************************** AS-API ****************************/

  add_service<srv::GetPanelInfo>(
    *this, node, services, "~/get_panel_info",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      TKrnxPanelInfo panel = {};
      const int code = krnx_GetPanelInfo(cont_no, req.arm_no, &panel);
      fill_krnx_result(resp.result, code, 0);
      resp.panel.error_lamp = panel.error_lamp;
      resp.panel.motor_lamp = panel.motor_lamp;
      resp.panel.cycle_lamp = panel.cycle_lamp;
      resp.panel.repeat_lamp = panel.repeat_lamp;
      resp.panel.run_lamp = panel.run_lamp;
      resp.panel.trigger_lamp = panel.trigger_lamp;
      resp.panel.teach_lock_lamp = panel.teach_lock_lamp;
      resp.panel.emergency = panel.emergency;
    });

  add_service<srv::SetPanelInfo>(
    *this, node, services, "~/set_panel_info",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (
        !is_undocumented_api_enabled(resp.result, "krnx_SetPanelInfo") ||
        !is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      TKrnxPanelInfo panel = {};
      panel.error_lamp = req.panel.error_lamp;
      panel.motor_lamp = req.panel.motor_lamp;
      panel.cycle_lamp = req.panel.cycle_lamp;
      panel.repeat_lamp = req.panel.repeat_lamp;
      panel.run_lamp = req.panel.run_lamp;
      panel.trigger_lamp = req.panel.trigger_lamp;
      panel.teach_lock_lamp = req.panel.teach_lock_lamp;
      panel.emergency = req.panel.emergency;
      RCLCPP_INFO(
        logger, "set_panel_info: arm_no:%d motor:%d cycle:%d run:%d", req.arm_no,
        panel.motor_lamp, panel.cycle_lamp, panel.run_lamp);
      const int code = krnx_SetPanelInfo(cont_no, req.arm_no, &panel);
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::GetMotionInfo>(
    *this, node, services, "~/get_motion_info",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::GetMotionInfo::Request;
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      int code = KRNX_NOERROR;
      switch (req.source)
      {
        case Req::SOURCE_CYCLIC:
        {
          TKrnxCurMotionData motion = {};
          code = krnx_GetCurMotionData(cont_no, req.arm_no, &motion);
          khi_msgs::msg::MotionInfo msg;
          msg.ang = to_vector<float>(motion.ang, KRNX_MAXAXES);
          msg.ang_ref = to_vector<float>(motion.ang_ref, KRNX_MAXAXES);
          msg.cur = to_vector<float>(motion.cur, KRNX_MAXAXES);
          msg.enc = to_vector<int64_t>(motion.enc, KRNX_MAXAXES);
          resp.data.push_back(msg);
          break;
        }
        case Req::SOURCE_AS:
        case Req::SOURCE_AS_CUR:
        case Req::SOURCE_AS_EX:
        {
          // One spare entry: the required size of these undocumented calls is unspecified.
          std::array<TKrnxMotionInfo, 2> motion = {};
          if (req.source == Req::SOURCE_AS)
          {
            code = krnx_GetMotionInfo(cont_no, req.arm_no, motion.data());
          }
          else if (req.source == Req::SOURCE_AS_CUR)
          {
            code = krnx_GetCurMotionInfo(cont_no, req.arm_no, motion.data());
          }
          else
          {
            code = krnx_GetMotionInfoEx(cont_no, req.arm_no, motion.data(), req.data_type);
          }
          resp.data.push_back(to_msg(motion[0]));
          break;
        }
        case Req::SOURCE_AS_SYNC:
        {
          if (req.data_num < 1 || req.data_num > MOTION_SYNC_MAX)
          {
            refuse(resp.result, "data_num must be 1-63 for SOURCE_AS_SYNC.");
            return;
          }
          std::vector<TKrnxMotionInfo> motion(MOTION_SYNC_MAX + 1);
          code = krnx_GetMotionInfoSync(cont_no, req.arm_no, motion.data(), req.data_num);
          for (int i = 0; i < req.data_num; i++)
          {
            resp.data.push_back(to_msg(motion[i]));
          }
          break;
        }
        default:
          refuse(resp.result, "Unknown source " + std::to_string(req.source) + ".");
          return;
      }
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::GetProgramInfo>(
    *this, node, services, "~/get_program_info",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      // Heap + one spare entry: 1.5 KiB struct, and ProgramInfo2's size is unspecified.
      std::vector<TKrnxProgramInfo> info(2);
      const int code = req.use_info2 ? krnx_GetProgramInfo2(cont_no, req.arm_no, info.data())
                                     : krnx_GetProgramInfo(cont_no, req.arm_no, info.data());
      fill_krnx_result(resp.result, code, 0);
      for (size_t arm = 0; arm < robot_.arms.size() && arm < KRNX_MAX_ROBOT; arm++)
      {
        khi_msgs::msg::MonInfo mon;
        mon.robot_status = info[0].mon[arm].robot_status;
        mon.monitor_speed = info[0].mon[arm].monitor_speed;
        mon.always_speed = info[0].mon[arm].always_speed;
        mon.accuracy = info[0].mon[arm].accuracy;
        resp.mon.push_back(mon);
        resp.robot.push_back(to_msg(info[0].robot[arm]));
      }
      for (const auto & pc : info[0].pc)
      {
        resp.pc.push_back(to_msg(pc));
      }
    });

  add_service<srv::GetIoInfo>(
    *this, node, services, "~/get_io_info",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::GetIoInfo::Request;
      int code = KRNX_NOERROR;
      if (req.source == Req::SOURCE_CYCLIC_EX)
      {
        TKrnxIoInfoEx io = {};
        code = krnx_GetCurIoInfoEx(cont_no, &io);
        resp.io_do = to_vector<uint8_t>(io.io_do, sizeof(io.io_do));
        resp.io_di = to_vector<uint8_t>(io.io_di, sizeof(io.io_di));
        resp.internal = to_vector<uint8_t>(io.internal, sizeof(io.internal));
      }
      else if (req.source == Req::SOURCE_CYCLIC || req.source == Req::SOURCE_AS)
      {
        // TKrnxIoInfo layout (512 signals), padded to the 960-signal size: krnx.h warns the
        // library may be built for either, and krnx_GetIoInfo is not in the manual.
        union
        {
          TKrnxIoInfo io;
          TKrnxIoInfoEx padding;
        } buffer = {};
        code = (req.source == Req::SOURCE_CYCLIC) ? krnx_GetCurIoInfo(cont_no, &buffer.io)
                                                  : krnx_GetIoInfo(cont_no, &buffer.io);
        resp.io_do = to_vector<uint8_t>(buffer.io.io_do, sizeof(buffer.io.io_do));
        resp.io_di = to_vector<uint8_t>(buffer.io.io_di, sizeof(buffer.io.io_di));
        resp.internal = to_vector<uint8_t>(buffer.io.internal, sizeof(buffer.io.internal));
      }
      else
      {
        refuse(resp.result, "Unknown source " + std::to_string(req.source) + ".");
        return;
      }
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::SetIo>(
    *this, node, services, "~/set_io",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      using Req = srv::SetIo::Request;
      const bool is_do = req.kind == Req::KIND_DO;
      if (!is_undocumented_api_enabled(resp.result, is_do ? "krnx_IoSetDO" : "krnx_IoSetDI"))
      {
        return;
      }
      if (!is_do && req.kind != Req::KIND_DI)
      {
        refuse(resp.result, "kind must be KIND_DO (0) or KIND_DI (1).");
        return;
      }
      if (req.data.empty() || req.data.size() != req.mask.size())
      {
        refuse(resp.result, "data and mask must be non-empty and the same length.");
        return;
      }
      if (req.data.size() > DO_MAX_SIGNAL / 8)
      {
        refuse(resp.result, "data is longer than 120 bytes (960 signals).");
        return;
      }
      RCLCPP_INFO(logger, "set_io: kind:%d bytes:%zu", req.kind, req.data.size());
      const auto * data = reinterpret_cast<const char *>(req.data.data());
      const auto * mask = reinterpret_cast<const char *>(req.mask.data());
      const int code = is_do ? krnx_IoSetDO(cont_no, data, mask, req.data.size())
                             : krnx_IoSetDI(cont_no, data, mask, req.data.size());
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::GetJointLimits>(
    *this, node, services, "~/get_joint_limits",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      float lower[KRNX_MAXAXES] = {};
      float upper[KRNX_MAXAXES] = {};
      int code = krnx_GetLimitM(cont_no, req.arm_no, lower);
      if (code == KRNX_NOERROR)
      {
        code = krnx_GetLimitP(cont_no, req.arm_no, upper);
      }
      fill_krnx_result(resp.result, code, 0);
      resp.lower = to_vector<float>(lower, KRNX_MAXAXES);
      resp.upper = to_vector<float>(upper, KRNX_MAXAXES);
    });

  add_service<srv::GetErrorCode>(
    *this, node, services, "~/get_error_code",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      int error_code = 0;
      const int code = krnx_GetErrorInfo(cont_no, req.arm_no, &error_code);
      fill_krnx_result(resp.result, code, 0);
      resp.error_code = error_code;
      if (error_code != 0)
      {
        convert_error_code(resp.as_error_code, error_code);
      }
    });

  add_service<srv::SetAsApiTimeout>(
    *this, node, services, "~/set_as_api_timeout",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (req.as_timeout <= 0)
      {
        refuse(resp.result, "as_timeout must be a positive number of milliseconds.");
        return;
      }
      const int code = krnx_SetAsApiTimeoutPeriod(cont_no, req.as_timeout);
      fill_krnx_result(resp.result, code, 0);
    });

  /**************************** RT-API (read-only + the two safe setters) ****************/

  add_service<srv::GetRtcState>(
    *this, node, services, "~/get_rtc_state",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      int first_error = KRNX_NOERROR;
      auto track = [&first_error](const int code)
      {
        if (code < 0 && first_error == KRNX_NOERROR)
        {
          first_error = code;
        }
        return code;
      };

      TKrnxRtcInfo info = {};
      track(krnx_GetRtcInfo(cont_no, &info));
      resp.cycle = info.cyc;
      resp.buffer = info.buf;
      resp.interpolation = info.interpolation;

      int rtc_switch = 0;
      track(krnx_GetRtcSwitch(cont_no, req.arm_no, &rtc_switch));
      resp.rtc_switch = (rtc_switch != 0);

      int error_flag = 0;
      track(krnx_GetRtcErrorFlag(cont_no, req.arm_no, &error_flag));
      resp.error_flag = error_flag;

      float comp[KRNX_MAXAXES] = {};
      track(krnx_GetRtcCompData(cont_no, req.arm_no, comp));
      resp.comp = to_vector<float>(comp, KRNX_MAXAXES);

      float comp_limit[KRNX_MAXAXES] = {};
      track(krnx_GetRtcCompLimit(cont_no, req.arm_no, comp_limit));
      resp.comp_limit = to_vector<float>(comp_limit, KRNX_MAXAXES);

      resp.buffer_length = track(krnx_GetRtcBufferLength(cont_no, req.arm_no));

      unsigned short sequence_no = 0;  // NOLINT(runtime/int): KRNX signature
      track(krnx_GetSequenceNo(cont_no, &sequence_no));
      resp.sequence_no = sequence_no;

      unsigned short krnx_kind = 0;  // NOLINT(runtime/int)
      unsigned short as_kind = 0;    // NOLINT(runtime/int)
      track(krnx_GetRtCyclicDataKind(cont_no, &krnx_kind, &as_kind));
      resp.cyclic_data_kind_krnx = krnx_kind;
      resp.cyclic_data_kind_as = as_kind;

      krnx_kind = 0;
      as_kind = 0;
      track(krnx_GetRtCyclicCtlKind(cont_no, &krnx_kind, &as_kind));
      resp.cyclic_ctl_kind_krnx = krnx_kind;
      resp.cyclic_ctl_kind_as = as_kind;

      int as_cycle = 0;
      track(krnx_GetASCycle(req.arm_no, &as_cycle));
      resp.as_cycle = as_cycle;

      fill_krnx_result(resp.result, first_error, 0);
    });

  add_service<srv::SetRtcCompMask>(
    *this, node, services, "~/set_rtc_comp_mask",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      if (is_active_.load() && !req.allow_while_active)
      {
        refuse(
          resp.result,
          "The hardware interface is active: masking a joint now makes it ignore the "
          "ros2_control position stream while the other joints keep moving. Deactivate the "
          "hardware component first, or set allow_while_active=true to override.");
        return;
      }
      RCLCPP_INFO(logger, "set_rtc_comp_mask: arm_no:%d mask:0x%X", req.arm_no, req.mask);
      const int code = krnx_SetRtcCompMask(cont_no, req.arm_no, req.mask);
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::SetRtcErrorFlag>(
    *this, node, services, "~/set_rtc_error_flag",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      RCLCPP_WARN(
        logger, "set_rtc_error_flag: arm_no:%d error_flag:%d (negative stops the robot, E1520)",
        req.arm_no, req.error_flag);
      // Shares the RT cyclic channel and its sequence number with write().
      std::lock_guard<std::mutex> lock(rtc_send_mutex_);
      const int code = krnx_SetRtcErrorFlag(cont_no, req.arm_no, req.error_flag, seq_no_);
      if (code == KRNX_NOERROR)
      {
        seq_no_++;
      }
      fill_krnx_result(resp.result, code, 0);
    });

  /**************************** Matrix-API / kinematics ****************************/

  add_service<srv::ForwardKinematics>(
    *this, node, services, "~/forward_kinematics",
    [this, cont_no, resolve_joints](const auto & req, auto & resp)
    {
      float joint[KRNX_MAXAXES] = {};
      if (
        !is_arm_valid(resp.result, req.arm_no) ||
        !resolve_joints(resp.result, req.arm_no, req.joint, joint))
      {
        return;
      }
      float xyzoat[UNSIZED_BUF] = {};
      float matrix[UNSIZED_BUF] = {};
      int code = krnx_JointToXyzoat(cont_no, req.arm_no, joint, xyzoat);
      if (code == KRNX_NOERROR)
      {
        code = krnx_JointToMatrix(cont_no, req.arm_no, joint, matrix);
      }
      fill_krnx_result(resp.result, code, 0);
      resp.joint = to_vector<float>(joint, KRNX_MAXAXES);
      std::copy(xyzoat, xyzoat + XYZOAT_SIZE, resp.xyzoat.begin());
      std::copy(matrix, matrix + MATRIX_SIZE, resp.matrix.begin());
    });

  add_service<srv::InverseKinematics>(
    *this, node, services, "~/inverse_kinematics",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::InverseKinematics::Request;
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }

      float old_joint[KRNX_MAXAXES] = {};
      const float * old_joint_ptr = old_joint;
      switch (req.reference)
      {
        case Req::REFERENCE_CURRENT:
          if (!get_current_joints(resp.result, req.arm_no, old_joint))
          {
            return;
          }
          break;
        case Req::REFERENCE_GIVEN:
          if (req.reference_joint.empty() || req.reference_joint.size() > KRNX_MAXAXES)
          {
            refuse(resp.result, "reference_joint must have 1-18 entries for REFERENCE_GIVEN.");
            return;
          }
          std::copy(req.reference_joint.begin(), req.reference_joint.end(), old_joint);
          break;
        case Req::REFERENCE_NONE:
          // NULL old_joint is documented for krnx_XyzoatToJoint / krnx_MatrixToJoint only.
          if (req.method != Req::METHOD_XYZOAT && req.method != Req::METHOD_MATRIX)
          {
            refuse(
              resp.result,
              "REFERENCE_NONE is only documented for METHOD_XYZOAT and METHOD_MATRIX; use "
              "REFERENCE_CURRENT or REFERENCE_GIVEN with this method.");
            return;
          }
          old_joint_ptr = nullptr;
          break;
        default:
          refuse(resp.result, "Unknown reference " + std::to_string(req.reference) + ".");
          return;
      }

      float joint[UNSIZED_BUF] = {};
      int code = KRNX_NOERROR;
      switch (req.method)
      {
        case Req::METHOD_XYZOAT:
          code = krnx_XyzoatToJoint(cont_no, req.arm_no, req.xyzoat.data(), joint, old_joint_ptr);
          break;
        case Req::METHOD_MATRIX:
          code = krnx_MatrixToJoint(cont_no, req.arm_no, req.matrix.data(), joint, old_joint_ptr);
          break;
        case Req::METHOD_XYZOAT_CONF:
          code = krnx_XyzoatToJoint2(
            cont_no, req.arm_no, req.xyzoat.data(), joint, old_joint_ptr, req.conf);
          break;
        case Req::METHOD_XYZ456:
          code = krnx_Xyz456ToJoint(
            cont_no, req.arm_no, req.xyzoat.data(), joint, old_joint_ptr, req.tool.data());
          break;
        default:
          refuse(resp.result, "Unknown method " + std::to_string(req.method) + ".");
          return;
      }
      fill_krnx_result(resp.result, code, 0);
      resp.joint = to_vector<float>(joint, KRNX_MAXAXES);
    });

  add_service<srv::Jacobian>(
    *this, node, services, "~/jacobian",
    [this, cont_no, resolve_joints](const auto & req, auto & resp)
    {
      float joint[KRNX_MAXAXES] = {};
      if (
        !is_arm_valid(resp.result, req.arm_no) ||
        !resolve_joints(resp.result, req.arm_no, req.joint, joint))
      {
        return;
      }
      float jacobian[UNSIZED_BUF] = {};
      float pose[UNSIZED_BUF] = {};
      int code = KRNX_NOERROR;
      if (req.tool_as_matrix)
      {
        const float * tool = req.tool_matrix.data();
        code = req.inverse ? krnx_JaInvMatrix(cont_no, req.arm_no, joint, tool, jacobian, pose)
                           : krnx_JacobiMatrix(cont_no, req.arm_no, joint, tool, jacobian, pose);
        std::copy(pose, pose + MATRIX_SIZE, resp.matrix.begin());
      }
      else
      {
        const float * tool = req.tool_xyzoat.data();
        code = req.inverse ? krnx_JaInvXyzoat(cont_no, req.arm_no, joint, tool, jacobian, pose)
                           : krnx_JacobiXyzoat(cont_no, req.arm_no, joint, tool, jacobian, pose);
        std::copy(pose, pose + XYZOAT_SIZE, resp.xyzoat.begin());
      }
      fill_krnx_result(resp.result, code, 0);
      std::copy(jacobian, jacobian + JACOBIAN_SIZE, resp.jacobian.begin());
    });

  add_service<srv::TransformMath>(
    *this, node, services, "~/transform_math",
    [this](const auto & req, auto & resp)
    {
      using Req = srv::TransformMath::Request;
      float xyzoat[UNSIZED_BUF] = {};
      float matrix[UNSIZED_BUF] = {};
      int code = KRNX_NOERROR;
      switch (req.op)
      {
        case Req::OP_MULTIPLY:
          code = req.use_matrix
                   ? krnx_MultiplyMatrix(req.matrix_a.data(), req.matrix_b.data(), matrix)
                   : krnx_MultiplyXyzoat(req.xyzoat_a.data(), req.xyzoat_b.data(), xyzoat);
          break;
        case Req::OP_INVERSE:
          code = req.use_matrix ? krnx_InverseMatrix(req.matrix_a.data(), matrix)
                                : krnx_InverseXyzoat(req.xyzoat_a.data(), xyzoat);
          break;
        case Req::OP_CONVERT:
          // Copy the input through so the "other representation" step below is uniform.
          if (req.use_matrix)
          {
            std::copy(req.matrix_a.begin(), req.matrix_a.end(), matrix);
          }
          else
          {
            std::copy(req.xyzoat_a.begin(), req.xyzoat_a.end(), xyzoat);
          }
          break;
        case Req::OP_FRAME:
          code = req.use_matrix
                   ? krnx_FrameMatrix(
                       req.matrix_a.data(), req.matrix_b.data(), req.matrix_c.data(),
                       req.matrix_d.data(), matrix)
                   : krnx_FrameXyzoat(
                       req.xyzoat_a.data(), req.xyzoat_b.data(), req.xyzoat_c.data(),
                       req.xyzoat_d.data(), xyzoat);
          break;
        default:
          refuse(resp.result, "Unknown op " + std::to_string(req.op) + ".");
          return;
      }
      // Fill the other representation of the result.
      if (code == KRNX_NOERROR)
      {
        code = req.use_matrix ? krnx_MatrixToXyzoat(matrix, xyzoat)
                              : krnx_XyzoatToMatrix(xyzoat, matrix);
      }
      fill_krnx_result(resp.result, code, 0);
      std::copy(xyzoat, xyzoat + XYZOAT_SIZE, resp.xyzoat.begin());
      std::copy(matrix, matrix + MATRIX_SIZE, resp.matrix.begin());
    });

  add_service<srv::GetConfig>(
    *this, node, services, "~/get_config",
    [this, cont_no, resolve_joints](const auto & req, auto & resp)
    {
      float joint[UNSIZED_BUF] = {};
      if (
        !is_arm_valid(resp.result, req.arm_no) ||
        !resolve_joints(resp.result, req.arm_no, req.joint, joint))
      {
        return;
      }
      int conf = 0;
      const int code = krnx_GetConfig(cont_no, req.arm_no, joint, &conf);
      fill_krnx_result(resp.result, code, 0);
      resp.joint = to_vector<float>(joint, KRNX_MAXAXES);
      resp.conf = conf;
    });

  /**************************** Undocumented: misc ****************************/

  add_service<srv::GetCycleCount>(
    *this, node, services, "~/get_cycle_count",
    [this, cont_no](const auto & req, auto & resp)
    {
      if (!is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      const int count = krnx_GetCycleCount(cont_no, req.arm_no, req.counter_no);
      fill_krnx_result(resp.result, count, 0);
      resp.count = count;
    });

  add_service<srv::SetJt1Mode>(
    *this, node, services, "~/set_jt1_mode",
    [this, logger](const auto & req, auto & resp)
    {
      if (!is_undocumented_api_enabled(resp.result, "krnx_SetJT1Mode"))
      {
        return;
      }
      RCLCPP_INFO(logger, "set_jt1_mode: mode:%d", req.mode);
      fill_krnx_result(resp.result, krnx_SetJT1Mode(req.mode), 0);
    });

  add_service<srv::SetConveyor>(
    *this, node, services, "~/set_conveyor",
    [this, cont_no, logger](const auto & req, auto & resp)
    {
      using Req = srv::SetConveyor::Request;
      if (
        !is_undocumented_api_enabled(resp.result, "krnx_SetConveyorSpeed/Pos") ||
        !is_arm_valid(resp.result, req.arm_no))
      {
        return;
      }
      RCLCPP_INFO(
        logger, "set_conveyor: arm_no:%d quantity:%d use_axis:%d axis:%d work:%d value:%.3f",
        req.arm_no, req.quantity, req.use_axis, req.axis_no, req.work_no, req.value);
      float previous = 0.0F;
      int code = KRNX_NOERROR;
      if (req.quantity == Req::QUANTITY_SPEED)
      {
        code = req.use_axis
                 ? krnx_SetConveyorSpeedEx(cont_no, req.arm_no, req.axis_no, req.value, &previous)
                 : krnx_SetConveyorSpeed(cont_no, req.arm_no, req.value, &previous);
      }
      else if (req.quantity == Req::QUANTITY_POSITION)
      {
        code = req.use_axis ? krnx_SetConveyorPosEx(
                                cont_no, req.arm_no, req.axis_no, req.work_no, req.value, &previous)
                            : krnx_SetConveyorPos(
                                cont_no, req.arm_no, req.work_no, req.value, &previous);
      }
      else
      {
        refuse(resp.result, "quantity must be QUANTITY_SPEED (0) or QUANTITY_POSITION (1).");
        return;
      }
      fill_krnx_result(resp.result, code, 0);
      resp.previous = previous;
    });

  add_service<srv::BufferStatus>(
    *this, node, services, "~/buffer_status",
    [this, cont_no](const auto & req, auto & resp)
    {
      resp.busy = krnx_BufferBusy(cont_no, req.buffer_no);
      resp.empty = krnx_BufferEmpty(cont_no, req.buffer_no);
      // Both return a state (>= 0) or a negative KRNX error code.
      fill_krnx_result(resp.result, std::min({resp.busy, resp.empty, 0}), 0);
    });

  add_service<srv::BufferSend>(
    *this, node, services, "~/buffer_send",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::BufferSend::Request;
      if (!is_undocumented_api_enabled(resp.result, "krnx_BufferSend*"))
      {
        return;
      }
      int code = KRNX_NOERROR;
      switch (req.type)
      {
        case Req::TYPE_FLOAT:
          code = krnx_BufferSendF(
            cont_no, req.buffer_no, req.request_code, req.float_data.data(),
            static_cast<int>(req.float_data.size()), req.timeout);
          break;
        case Req::TYPE_WORD:
          code = krnx_BufferSendW(
            cont_no, req.buffer_no, req.request_code, req.word_data.data(),
            static_cast<int>(req.word_data.size()), req.timeout);
          break;
        case Req::TYPE_BYTE:
          code = krnx_BufferSendB(
            cont_no, req.buffer_no, req.request_code,
            reinterpret_cast<const char *>(req.byte_data.data()),
            static_cast<int>(req.byte_data.size()), req.timeout);
          break;
        default:
          refuse(resp.result, "Unknown type " + std::to_string(req.type) + ".");
          return;
      }
      fill_krnx_result(resp.result, code, 0);
    });

  add_service<srv::BufferReceive>(
    *this, node, services, "~/buffer_receive",
    [this, cont_no](const auto & req, auto & resp)
    {
      using Req = srv::BufferReceive::Request;
      if (!is_undocumented_api_enabled(resp.result, "krnx_BufferRecv*/Peek*"))
      {
        return;
      }
      if (req.max_count < 1 || req.max_count > BUFFER_RECV_MAX)
      {
        refuse(resp.result, "max_count must be 1-4096.");
        return;
      }
      // Whether `num` is in/out or out-only is unspecified: pass the wanted count in, give the
      // library far more room than that, and clamp what comes back to the real capacity.
      short request_code = 0;  // NOLINT(runtime/int): KRNX signature
      int num = req.max_count;
      int code = KRNX_NOERROR;
      auto received = [&num]() { return std::clamp(num, 0, BUFFER_RECV_CAPACITY); };
      switch (req.type)
      {
        case Req::TYPE_FLOAT:
        {
          std::vector<float> data(BUFFER_RECV_CAPACITY);
          code = req.peek ? krnx_BufferPeekF(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout)
                          : krnx_BufferRecvF(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout);
          resp.float_data.assign(data.begin(), data.begin() + received());
          break;
        }
        case Req::TYPE_WORD:
        {
          std::vector<short> data(BUFFER_RECV_CAPACITY);  // NOLINT(runtime/int)
          code = req.peek ? krnx_BufferPeekW(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout)
                          : krnx_BufferRecvW(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout);
          resp.word_data.assign(data.begin(), data.begin() + received());
          break;
        }
        case Req::TYPE_BYTE:
        {
          std::vector<char> data(BUFFER_RECV_CAPACITY);
          code = req.peek ? krnx_BufferPeekB(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout)
                          : krnx_BufferRecvB(
                              cont_no, req.buffer_no, &request_code, data.data(), &num, req.timeout);
          resp.byte_data.assign(data.begin(), data.begin() + received());
          break;
        }
        default:
          refuse(resp.result, "Unknown type " + std::to_string(req.type) + ".");
          return;
      }
      fill_krnx_result(resp.result, code, 0);
      resp.request_code = request_code;
      if (code != KRNX_NOERROR)
      {
        resp.float_data.clear();
        resp.word_data.clear();
        resp.byte_data.clear();
      }
    });

  RCLCPP_INFO(
    logger, "KRNX API services: %zu created (state-changing undocumented APIs %s)",
    services.size(), robot_.enable_undocumented_krnx_api ? "ENABLED" : "disabled");
}

}  // namespace khi_hardware
