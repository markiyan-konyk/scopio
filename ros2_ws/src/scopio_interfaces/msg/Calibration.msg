# The microscope's spatial calibration, owned by calibration_node and published
# as a latched (transient-local) topic so late-joining clients get it
# immediately. Persisted to disk so it survives restarts.
#   um_per_px      : image scale (micrometres per native camera pixel)
#   steps_per_um_* : stage conversion (Sangaboard steps per micrometre per axis)
std_msgs/Header header
bool has_um_per_px
float64 um_per_px
float64 steps_per_um_x
float64 steps_per_um_y
float64 steps_per_um_z
