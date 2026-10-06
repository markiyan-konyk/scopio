# Stage position, open-loop: the origin is wherever the stage was when stage_node started.
std_msgs/Header header
bool connected
int32 x          # steps
int32 y          # steps
int32 z          # steps
float32 x_um     # micrometres (steps / steps_per_um, which is 1.0 until calibrated)
float32 y_um
float32 z_um
