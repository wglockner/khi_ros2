# Joint motion data, KRNX_MAXAXES (18) entries each. Units: rad (rotational) / mm (linear).
# Fields a given source does not provide are left empty.
float32[] ang       # actual joint positions
float32[] ang_ref   # commanded joint positions
float32[] vel       # actual joint velocities   (AS-API sources only)
int64[] vel_ref     # commanded velocity, raw   (AS-API sources only)
float32[] cur       # actual joint current [A]  (RT cyclic source only)
float32[] cur_ref   # commanded joint current   (AS-API sources only)
int64[] enc         # actual encoder values [bit]
