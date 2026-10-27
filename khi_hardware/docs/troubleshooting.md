## Troubleshooting

### Error: Unable to reset error.
Refer to the [Troubleshooting Manual] for resolution.

### Error: AS ERROR controller_no:XX arm_no:XX error_code:XX
Error occurred during the real-time control.  
Please refer to the [AS Language Reference Manual] to resolve the error.

### Error: RTC SWITCH turned OFF controller_no:XX arm_no:XX
Real-time control of the robot controller turned OFF. It needs to restart to control again.

### Error: Program 'rb_rtcl' is not running.
Please perform one of the following actions.  
(a) After stopping real-time control with Ctrl+C, restart real-time control.  
(b) Transition to Activate using SetHardwareComponentState Service.
```
ros2 control set_hardware_component_state <robot_name> active
```

### Error: Communication with the robot controller has been lost.
Please follow the steps below to recover:  
1. Transition the state of the robot from `unconfigured` to `inactive`:
```
ros2 control set_hardware_component_state <robot_name> inactive
```
2. Transition the state of the `khi_controller` from `active` to `inactive` and then back to `active`:
```
ros2 control switch_controllers --deactivate khi_controller  
ros2 control switch_controllers --activate khi_controller
```
3. Transition the state of the robot to `active`:
```
ros2 control set_hardware_component_state <robot_name> active
```

### Error: krnx_Open returned -0x1100 / 'connect' failed (errno=111 port=8100) on a SECOND controller
The controller pings, is in REPEAT and shows no error, and `controller_no` 0 on the other
arm works. KRNX maps each controller number to a port set through `port_ref.tbl` /
`port_def.tbl`, read from the **working directory** of the process calling `krnx_Open`.
Without the files only controller number 0 gets the E-controller ports (AUX/telnet 23);
controller number 1 gets a set with AUX 8100 that a real E-controller does not serve, so
the connect is refused. Ship the tables and start the controller manager in their
directory (the rambl_compositions launcher does this for `krnx_port_tables:`; see
`isu_workcell/config/krnx/README.md`). `port_ref.tbl` needs one line per controller slot
(16); fewer lines are rejected (`bad ctrl no(N)`) and the defaults are silently used.
Check with the probe: `tools/krnx_probe.cpp` built by hand, `probe <ip> read 1` from the
table directory. Seen 2026-09-29 on the CX165L (192.168.1.23).

### Error: XX returned -0xXXXX
API %s of KRNX returned the error code -0x%X. (e.g. `krnx_Open returned -0x%X`)  
Please refer to the KRNX Error section to resolve the error.

### Error: A commanded position exceeding the speed limit was sent. JTXX cmd:XX old_cmd:XX cmd_spd:XX act:XX
After releasing the emergency stop, if the above error occurs at the moment the robot transitions to `active`, please follow the steps below to address it.
1. Transition the state of the robot to `inactive`:
```
ros2 control set_hardware_component_state <robot_name> inactive
```
2. Transition the state of the `khi_controller` from `active` to `inactive` and then back to `active`:
```
ros2 control switch_controllers --deactivate khi_controller  
ros2 control switch_controllers --activate khi_controller
```
3. Transition the state of the robot to `active`:
```
ros2 control set_hardware_component_state <robot_name> active
```

### Warning: The current is saturated in JTXX. Please reduce the acceleration or change the motion.
To prevent failure of the robot, please ensure that this warning does not appear.  
If you have not performed a warm-up operation, please do so.  
If the weight setting has not been configured, please do so.  
If the load mass and load center of gravity position for the tool have not been set, please configure them.  
Instead of the joint experiencing current saturation, it may be improved by reducing the acceleration of the joint that is operated simultaneously.

<br>

### Error: Cannot turn on motor power: N x 'ZPOW ON' over X s never lit the motor lamp

activate() sends the AS monitor command `ZPOW ON` itself (not `krnx_PowerOnMotorWithOnWait`,
which discards the controller's reply and gives up after one ~2.4 s wait), logs the
controller's verbatim reply per attempt, polls the panel motor lamp for `MOTOR_ON_WAIT_MS`,
and retries `MOTOR_ON_ATTEMPTS` times. If it still fails the log carries the last reply and a
full `Controller status (...)` line.

- Reply non-empty (a `(Wxxxx)` / `(Exxxx)` message): that is the controller's reason.
  Clear it (pendant message line) and activate again.
- Reply empty, no AS error, and the pendant CAN turn motor power on: something else was on
  the KRNX AS channel during activate. The driver reserves that channel while activating
  (`KhiDriver::is_transitioning()`: publishers skip, services refuse, and `k2_khi_panel` holds
  its polls), so look for another KRNX client on the controller (a second driver, a probe).
- Reply empty and the pendant cannot turn motor power on either: a cell interlock or a
  safety-unit (Cubic-S) condition that AS does not report as an error. Check the pendant.

Seen on the K2 cell 2026-09-19: two consecutive bring-ups refused with a clean controller;
a fresh read-only KRNX session (`tools/krnx_probe.cpp`) then had `ZPOW ON` accepted
instantly, which pointed at concurrent channel use during activate rather than the controller.

## KRNX Error

Error code of KRNX API is defined in “khi_hardware/include/krnx.h”.   
Description and Value of the error codes are shown in the table below.

| Macro Definition          | Description                     | Value     |
| ------------------------- | ------------------------------- | --------- |
| KRNX_NOERROR              | No Error                        | (0x0000)  |
| KRNX_E_BADARGS            | Invalid Argument                | (-0x1000) |
| KRNX_E_INTERNAL           | Internal Error                  | (-0x1001) |
| KRNX_E_NOTSUPPORTED       | Not Supported API               | (-0x1002) |
| KRNX_E_TIMEOUT            | Timeout                         | (-0x1003) |
| KRNX_E_AUXNOTREADY        | AUX Monitor Not Ready           | (-0x1004) |
| KRNX_E_FOPENFAIL          | File Open Fail                  | (-0x1005) |
| KRNX_E_FILENOTREADY       | File Not Exist                  | (-0x1006) |
| KRNX_E_MATRIX             | Matrix Calculation Error        | (-0x1007) |
| KRNX_E_OUTOFRANGE         | Inverse Conversion Error        | (-0x1008) |
| KRNX_E_CANNOTCAL          | Inverse Jacobian Error          | (-0x1009) |
| KRNX_E_COMPDATA           | RTC Error                       | (-0x100a) |
| KRNX_E_BADUSERID          | Bad User ID                     | (-0x100c) |
| KRNX_E_NULLRESP           | Data Not Received               | (-0x100d) |
| KRNX_E_LOSTPROMPT         | Timeout for Prompt Receive      | (-0x100e) |
| KRNX_E_BUFSND             | Communication Send Error        | (-0x1010) |
| KRNX_E_BUFRCV             | Communication Receive Error     | (-0x1011) |
| KRNX_E_BUFTMO             | Communication Timeout           | (-0x1012) |
| KRNX_E_ASERROR            | AS Error                        | (-0x1020) |
| KRNX_E_NOROBOT            | No Robot Setting                | (-0x1021) |
| KRNX_E_SOCK               | Socket Create Error             | (-0x2000) |
| KRNX_E_NOHOST             | Bad Hostname                    | (-0x2001) |
| KRNX_E_IOCTLSOCK          | Socket Setting Error            | (-0x2002) |
| KRNX_E_SOCKWRITE          | Socket Write Error              | (-0x2003) |
| KRNX_E_SOCKREAD           | Socket Read Error               | (-0x2004) |
| KRNX_E_NODATA             | No Socket Data                  | (-0x2005) |
| KRNX_E_INVALIDPORT        | Invalid Port Number             | (-0x2006) |
| KRNX_E_CONNECT            | Socket Connect Fail             | (-0x2007) |
| KRNX_E_CANTLOGIN          | Login Fail                      | (-0x2008) |
| KRNX_E_ALREADYOPENED      | Socket Already Used             | (-0x2009) |
| KRNX_E_UNEXPECTED         | Received Data Error             | (-0x2010) |
| KRNX_E_KINENOTREADY       | Kinematics Not Initialized      | (-0x2011) |
| KRNX_E_ASDELAYED          | Communication Sync Error        | (-0x2012) |
| KRNX_E_BUFEMPTY           | Communication Buffer Error      | (-0x2013) |
| KRNX_E_BUFNO              | Invalid Buffer Number           | (-0x2014) |
| KRNX_E_BUFDATANUM         | Send Data Error                 | (-0x2015) |
| KRNX_E_RT_INTERNAL        | RT Communication Internal Error | (-0x2100) |
| KRNX_E_RT_CONNECT         | RT Communication Connect Error  | (-0x2101) |
| KRNX_E_RT_TIMEOUT         | RT Communication Timeout        | (-0x2102) |
| KRNX_E_RT_NOTCONNECT      | RT Communication Connect Error  | (-0x2103) |
| KRNX_E_RT_SEND            | RT Communication Send Error     | (-0x2104) |
| KRNX_E_PCASALREADYRUNNING | PC-AS Already Running           | (-0x2200) |
| KRNX_E_TOOMANYPROC        | Too Many Process                | (-0x2201) |
| KRNX_E_INVALIDFILENAME    | Invalid File Name               | (-0x2202) |
| KRNX_E_ILLCONTNO          | Invalid Controller Number       | (-0x2203) |
| KRNX_E_UNDEFF             | Undefined Error                 | (-0xFFFF) |

## ros2_control features the driver uses (2026-10-27)

- **Introspection.** The component registers its driver internals with the controller_manager's
  pal_statistics registry (`REGISTER_ROS2_CONTROL_INTROSPECTION`, `KhiDriverHealth` in
  `khi_driver.hpp`): `<component>.khi.rtc_backlog` (deepest RTC queue of any arm, points),
  `rtc_refusals` (compensation points the controller refused, cumulative), `rtc_seq`,
  `write_errors`, `read_failures`, `feedback_frozen` (1 while the robot->PC stream is frozen),
  `read_us` / `write_us` (cycle durations), `sim_step_refusals` (mock only). Published at the loop
  rate on `/controller_manager/introspection_data/{names,values}`; `print_recorder.py` saves them
  per run (`introspection.csv`). PlotJuggler reads the same topics.
- **Async component.** `<ros2_control ... is_async="true" rw_rate="100" thread_priority="50">`
  (isu_ros2_control macro params `is_async`, `thread_priority`; the dual cell sets it for the
  CX165L through `grind_async`) runs read()/write() on the component's own thread, triggered each
  controller_manager cycle, so a slow or stalled KRNX call on one controller cannot delay the
  other's RTC stream. `rw_rate` must stay the update rate: the driver derives the RTC cycle it
  declares to the Kawasaki from it. libkrnx serialises its API calls on one global mutex
  (`dll_ApiMutex`), so concurrent calls from the two threads are safe but a long call still holds
  the other up; the hot path (`krnx_GetCurMotionDataEx`, `krnx_PrimeRtcCompData`,
  `krnx_SendRtcCompData`) is non-blocking. Verified on the mock only; `grind_async` defaults to
  false since the first real grinder move after the async launch (2026-10-27 14:02:45 CDT) ended
  0.77 s in with `rtc_active 0`, `rb_rtc1` not running, no error lamp, no emergency / protective
  stop flag. The motor_lamp / rtc_active change logged afterwards was the driver's own deactivate.
  Established the same afternoon with the diagnostics and a passive capture: the base had been
  planned past its mechanical hard stop (see the joint limit note above); neither the thread nor
  the controller.
  That deactivate also blocked the control loop for 1.59 s (both arms); fixed the same day, below.
- **Non-blocking stop sequences (2026-10-27).** When `write()` returns DEACTIVATE or ERROR the
  resource manager runs the component's lifecycle callback inside the control loop. The KRNX
  stop sequence (1 s HOLD grace, hold, kill program, motor power off; ~1.6 s, and every call
  times out when the link is gone) now runs on a worker thread started by `on_deactivate` /
  `on_error`, which return at once (`start_transition_worker` in `khi_hardware_interface.cpp`).
  While it runs `read()` / `write()` skip (the state interfaces keep their last value, the
  publishers and services already hold off through `is_transitioning()`); configure / activate /
  cleanup / shutdown join it first, so the controller is never driven from two threads, and a
  failed sequence is logged by the worker and warned about at the next activation. Verified on
  the mock with `sim_transition_s:=1.5`: deactivating the grinder while the CX110L moved left no
  overrun and the move completed; reactivation joined the sequence and the grinder moved again.
- **RTC stop diagnostics (2026-10-27).** The first time `write()` finds an arm not writable
  (`rtc_active 0`) the driver logs `RTC stop diagnostics`: the whole status word (repeat_lamp 0 =
  TEACH, run_lamp 0 = HOLD, cycle / trigger / teach-lock lamps, emergency, system_emergency,
  protective_stop, enverr_warm, can_send_cmd_pos, monitor speed), the panel lamps and the AS
  error list, or an explicit "the controller reports NO error" line. One dump per episode.
  Untested against a real stop (nothing in the mock can produce one).
- **Joint limit enforcement.** With `enforce_command_limits: true` on the controller_manager the
  resource manager clamps every claimed position command into the URDF joint range and limits its
  step from the previous command to the URDF velocity limit x the cycle (ros2_control 4.28+,
  `joint_limits_helpers.cpp`). The URDF velocity limits are therefore the controller's RTC
  compensation step limits (the `spd_limit` values this driver logs on a refusal) less a 2 %
  margin (`cx110l_controller_limits.xacro`, `cx165l_macro.xacro`): a step the limiter saturates at
  exactly the controller's limit could still be refused by float rounding, and one refused point
  holds the previous one, after which every later point is refused too and the arm freezes while
  commands stream. The limiter THROWS, and the controller_manager dies, when a MEASURED position is
  more than 0.5 deg (0.0087 rad) outside the URDF range, so the URDF ranges of both arms are the
  controllers' own working ranges, EXCEPT where the URDF limit is a physical hard stop the arm can
  rest on: the CX165L's joint 1 stops mechanically at about +2.6 deg (encoder) in this cell, its URDF
  upper limit stays +1 deg, and the limiter is disabled for that joint (`<limits enable="false"/>`,
  macro param `unenforced_joints`). Widening it to +160 deg on 2026-10-27 drove the base into the
  stop three times ("Deviation error of JT1": five joints tracked the command, JT1 moved 0.03 deg
  and stopped). A clamp logs `Command of at least one joint is out of limits` (throttled); the
  JTC still reports its goal as reached.
- **Mock fidelity.** `KhiMockDriver` (sim_plugin:=khi) follows the command through a first-order
  lag of `sim_lag_s` (0.09 s, the 80..94 ms the CX110L measured on 2026-10-22; 0 = measured
  equals commanded) and refuses a command step past the URDF velocity limit x cycle exactly like
  the controller (holds the previous point, logs `[SIM] A commanded position exceeding the speed
  limit`, counts `khi.sim_step_refusals`). Measured on the mock: lag median 85 ms, p10 79, p90 91.
