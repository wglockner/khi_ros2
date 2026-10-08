# Common result of a KRNX API call, embedded in the extended KRNX service responses.

# true when the KRNX return code is KRNX_NOERROR and the controller reported no AS error
bool success

# KRNX return code (0 = KRNX_NOERROR, negative = error). See khi_hardware/docs/troubleshooting.md.
int32 krnx_code

# krnx_code as hex text (e.g. "-0x1003"), same notation as troubleshooting.md. Empty on success.
string krnx_err

# AS (controller) error code, only set by AUX-API calls. 0 = none.
# -1xxxx Operation Error / -2xxxx Warning / -3xxxx Error / -4xxxx Fatal Error
int32 as_error_code

# Human readable detail: the AS error text, or the reason the driver refused the request.
string error_msg
