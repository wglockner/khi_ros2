cmake_minimum_required(VERSION 3.5)
project(khi_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to Cﭞ
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/ErrorInfo.msg"
  "msg/ActualCurrent.msg"
  "msg/KrnxResult.msg"
  "msg/MonInfo.msg"
  "msg/MotionInfo.msg"
  "msg/PanelInfo.msg"
  "msg/StepperInfo.msg"
)
set(srv_files
  "srv/GetSignal.srv"
  "srv/ExecKhiCommand.srv"
  "srv/SetSignal.srv"
  "srv/ResetError.srv"
  "srv/SetTimeout.srv"
  "srv/SetATISoftwareBias.srv"
  "srv/ChangeFTOutputMode.srv"
  # Extended KRNX API services (khi_hardware/docs/krnx_api_services.md)
  "srv/BufferReceive.srv"
  "srv/BufferSend.srv"
  "srv/BufferStatus.srv"
  "srv/DeleteData.srv"
  "srv/DoInstruction.srv"
  "srv/ForwardKinematics.srv"
  "srv/GetConfig.srv"
  "srv/GetCycleCount.srv"
  "srv/GetErrorCode.srv"
  "srv/GetFrame.srv"
  "srv/GetIoInfo.srv"
  "srv/GetJointLimits.srv"
  "srv/GetMotionInfo.srv"
  "srv/GetPanelInfo.srv"
  "srv/GetProgramInfo.srv"
  "srv/GetRtcState.srv"
  "srv/GetSignalAux.srv"
  "srv/InverseKinematics.srv"
  "srv/Jacobian.srv"
  "srv/ListData.srv"
  "srv/LoadData.srv"
  "srv/MotorPower.srv"
  "srv/ProgramControl.srv"
  "srv/RecordStep.srv"
  "srv/ResetSignals.srv"
  "srv/SaveData.srv"
  "srv/SetAsApiTimeout.srv"
  "srv/SetConveyor.srv"
  "srv/SetFrame.srv"
  "srv/SetIo.srv"
  "srv/SetJt1Mode.srv"
  "srv/SetMonitorSpeed.srv"
  "srv/SetPanelInfo.srv"
  "srv/SetRtcCompMask.srv"
  "srv/SetRtcErrorFlag.srv"
  "srv/SetSignalAux.srv"
  "srv/TransformMath.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
