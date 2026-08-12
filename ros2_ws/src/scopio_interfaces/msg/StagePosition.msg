# The microscope's global ("in the map") position, accumulated from relative
# moves (open-loop; origin = wherever the stage was at startup). Reported in
# both raw Sangaboard steps and calibrated micrometres (via the calibration
# store -- steps_per_um). Jump-scanning uses the step values.
std_msgs/Header header
bool connected
int32 x          # steps
int32 y          # steps
int32 z          # steps
float32 x_um     # micrometres (steps / steps_per_um); 0 if uncalibrated
float32 y_um
float32 z_um
