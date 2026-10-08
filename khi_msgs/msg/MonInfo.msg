# TKrnxMonInfo (KRNX API Manual 9.6)
uint16 STATUS_ERROR = 1
uint16 STATUS_MOTOR_POWER_ON = 2
uint16 STATUS_REPEAT_MODE = 4
uint16 STATUS_WORKING_AS_REPEAT = 8
uint16 STATUS_CHECK_MODE = 16

int16 robot_status     # OR of STATUS_*
float32 monitor_speed  # [%]
float32 always_speed   # program speed set with ALWAYS [%]
float32 accuracy
