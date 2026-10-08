# TKrnxStepperInfo (KRNX API Manual 9.6)
uint16 STATUS_WAIT = 1
uint16 STATUS_EXECUTING = 2
uint16 STATUS_STOP = 4

int16 status           # OR of STATUS_*
int64 exec_count
int64 remain_count     # -1: unlimited
string program_name
int16 priority
int16 step_number
string step_name       # executing program instruction
