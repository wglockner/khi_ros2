# Extended KRNX API services

`khi_hardware` exposes the KRNX API as ROS 2 services on the existing service node,
`/khi_controller<controller_no>/khi_service/<name>`, next to `get_signal`, `set_signal`,
`exec_khi_command`, `reset_error`, `set_timeout`, `set_ati_software_bias` and
`change_ft_output_mode`. Implementation: `src/khi_krnx_api_services.cpp`. Reference: KRNX API
Manual 90210-1416DEC (DEV 2.3.0); the shipped library is 3.4.0, so part of its API is newer
than the manual.

Every response starts with `khi_msgs/KrnxResult result`:

| field | meaning |
|---|---|
| `success` | KRNX return code >= 0 **and** no AS error |
| `krnx_code` / `krnx_err` | raw KRNX return code, and as hex (`-0x1003`); see [troubleshooting.md](troubleshooting.md) |
| `as_error_code` | controller (AS) error code, AUX-API calls only |
| `error_msg` | AS error text, or why the driver refused the request |

Conventions: `arm_no` is 0-based. Units are KRNX native and unconverted: **mm** and **rad**.
XYZOAT = X,Y,Z [mm] + O,A,T [rad], Euler Z-Y-Z. A matrix is 12 floats, column-major:
n(xyz) o(xyz) a(xyz) p(xyz).

An RViz front end for all of these lives in the cell repo: `k2_craft/k2_khi_panel`
(`k2_khi_panel/K2KhiPanel`).

## Rules

1. **The driver owns the connection and the RTC stream.** Functions that open/close the
   connection or push data into the RT cyclic channel are not services (see "Not exposed").
2. Functions in the manual are always available.
3. Undocumented functions that only read are always available (output buffers oversized).
4. Undocumented functions that **change controller state** refuse unless the hardware
   parameter is set in the robot's `<ros2_control>` `<hardware>` block (optional, default false):

   ```xml
   <param name="enable_undocumented_krnx_api">true</param>
   ```

KRNX AUX/AS/RT APIs are thread-safe but not reentrant (manual ch. 11); the services run on the
single-threaded `khi_service` executor beside the control loop.

While `activate()` or `deactivate()` runs (motor power-on / RTC start; hold / kill / power-off) every service refuses with
`KrnxResult.error_msg = "... refused: hardware activation in progress ..."`, and the publishers
skip their cycle: the power-on sequence is a monitor command followed by panel-lamp reads, and
a concurrent AS call can make it wrongly report a refusal. Retry once the component is active.

## Services

Doc: M = in the manual, U = undocumented read-only, **G** = undocumented, gated by the parameter.

| service | type | KRNX functions | doc |
|---|---|---|---|
| `program_control` | ProgramControl | Execute, Prime, Hold, Continue, Abort, Kill, PcExecute, PcContinue, PcAbort, PcKill, PcEnd | M |
| `motor_power` | MotorPower | PowerOnMotorWithOnWait, PowerOffMotorWithOffWait | U |
| `set_monitor_speed` | SetMonitorSpeed | SetMonSpeed | M |
| `do_instruction` | DoInstruction | Do — **motion instructions move the robot** | M |
| `record_step` | RecordStep | RecOneStep | M |
| `get_frame` / `set_frame` | GetFrame / SetFrame | Get/SetToolMatrix, Get/SetBaseMatrix | M |
| `get_signal_aux` / `set_signal_aux` | GetSignalAux / SetSignalAux | GetSignal, SetSignal (960-signal range) | M |
| `reset_signals` | ResetSignals | Reset — all external outputs OFF | M |
| `delete_data` / `list_data` | DeleteData / ListData | Delete, List | M |
| `save_data` / `load_data` | SaveData / LoadData | Save, LoadEx (files on the driver host) | U |
| `get_panel_info` | GetPanelInfo | GetPanelInfo | M |
| `set_panel_info` | SetPanelInfo | SetPanelInfo | **G** |
| `get_motion_info` | GetMotionInfo | GetCurMotionData (M); GetMotionInfo, GetCurMotionInfo, GetMotionInfoEx, GetMotionInfoSync | M / U |
| `get_program_info` | GetProgramInfo | GetProgramInfo (M), GetProgramInfo2 | M / U |
| `get_io_info` | GetIoInfo | GetCurIoInfoEx, GetCurIoInfo (M); GetIoInfo | M / U |
| `set_io` | SetIo | IoSetDO, IoSetDI | **G** |
| `get_joint_limits` | GetJointLimits | GetLimitM, GetLimitP | M |
| `get_error_code` | GetErrorCode | GetErrorInfo, ConvertErrorCode | M |
| `set_as_api_timeout` | SetAsApiTimeout | SetAsApiTimeoutPeriod | M |
| `get_rtc_state` | GetRtcState | GetRtcInfo, GetRtcSwitch, GetRtcErrorFlag, GetRtcCompData, GetRtcCompLimit, GetRtcBufferLength, GetRtCyclicDataKind (M); GetSequenceNo, GetRtCyclicCtlKind, GetASCycle | M / U |
| `set_rtc_comp_mask` | SetRtcCompMask | SetRtcCompMask — refused while active unless `allow_while_active` | M |
| `set_rtc_error_flag` | SetRtcErrorFlag | SetRtcErrorFlag — negative flag **stops the robot** (E1520) | M |
| `forward_kinematics` | ForwardKinematics | JointToXyzoat, JointToMatrix | M |
| `inverse_kinematics` | InverseKinematics | XyzoatToJoint, MatrixToJoint (M); XyzoatToJoint2, Xyz456ToJoint | M / U |
| `jacobian` | Jacobian | JacobiXyzoat, JacobiMatrix, JaInvXyzoat, JaInvMatrix (6x6 element order unverified) | U |
| `transform_math` | TransformMath | Multiply/Inverse/Frame Xyzoat+Matrix, MatrixToXyzoat, XyzoatToMatrix (offline math) | M |
| `get_config` | GetConfig | GetConfig | U |
| `get_cycle_count` | GetCycleCount | GetCycleCount | U |
| `set_jt1_mode` | SetJt1Mode | SetJT1Mode | **G** |
| `set_conveyor` | SetConveyor | SetConveyorSpeed(Ex), SetConveyorPos(Ex) | **G** |
| `buffer_status` | BufferStatus | BufferBusy, BufferEmpty | U |
| `buffer_send` / `buffer_receive` | BufferSend / BufferReceive | BufferSendF/W/B, BufferRecvF/W/B, BufferPeekF/W/B | **G** |

Kinematics are for the **flange in the robot null-base frame**: the controller's TOOL/BASE are
not applied (manual 7.1); compose them with `get_frame` + `transform_math`.

```bash
export ROS_DOMAIN_ID=10
S=/khi_controller0/khi_service
ros2 service call $S/get_frame khi_msgs/srv/GetFrame "{frame: 0, arm_no: 0}"            # TOOL
ros2 service call $S/forward_kinematics khi_msgs/srv/ForwardKinematics "{arm_no: 0}"   # current joints
ros2 service call $S/get_rtc_state khi_msgs/srv/GetRtcState "{arm_no: 0}"
ros2 service call $S/get_joint_limits khi_msgs/srv/GetJointLimits "{arm_no: 0}"
```

## Not exposed

| functions | why |
|---|---|
| `krnx_Open`, `krnx_eth_init/open/close`, `krnx_RtcInit` | The driver owns the connection (`krnx_OpenSingleProcess`) and RTC setup; a second open/close/init under a live control loop tears the session down. |
| `krnx_SetRtcCompData`, `krnx_SetRtcCompDataEx`, `krnx_SendRtCtlData`, `krnx_PrimeRtIoData`, `krnx_PrimeCommandPosition` | They write the RT cyclic channel `write()` streams motion on (`krnx_SendRtcCompData` is `krnx_SendRtCtlData(seq, KIND_ANGLE_RELATIVE)`). A service thread would inject compensation data into a moving arm. Real-time I/O and absolute position commands must be integrated **into `write()`**, and `TKrnxRtIoCtl*` (`sync`, `command`) is undocumented in manual DEV 2.3.0 — needs the 3.x manual first. |
| `krnx_SaveEx` | `krnx_Save` is `krnx_SaveEx` with a NULL callback; the callback's return-value contract is undocumented. |
| `krnx_NotSupport`, `krnx_GetArmMode` | Stubs in libkrnx 3.4.0 (constant return). |
| `krnx_PanelHw`, `krnx_PanelToPC`, `krnx_IoGetDDSig`, `krnx_DDSigInfo`, `krnx_GetDebugInfo(Sync)`, `krnx_SetAppParam`, `krnx_SetJoint` | Undocumented with unnamed or unsized parameters (legacy PC-controller compatibility); no safe way to size buffers or know the effect. Panel state is available through `get_panel_info`. |
| `krnx_Arc*` (16), `krnx_RunPcAs`, `krnx_StopPcAs(Ex)`, `krnx_SetPriority`, `krnx_timer_callback` | Declared in `krnx.h` but **not exported** by the shipped `libkrnx.so`: cannot be linked. |

## Tests

`test/test_krnx_api_services.cpp` runs offline (the driver is never opened): Matrix-API
results, refusal paths, and clean failure of controller calls without a connection.

```bash
colcon test --packages-select khi_hardware --event-handlers console_direct+
```

Nothing here has been exercised against a real controller yet: verify the read-only services
first (`get_rtc_state`, `get_joint_limits`, `get_frame`, `forward_kinematics` against the
pendant), with the arm idle, before using any state-changing service.
