# Current camera configuration + the REAL measured frame rate. Published by
# camera_node so any client can read the live settings (replaces the monolith
# /get_camera_controls + measured-fps). measured_fps comes from the sensor's
# FrameDuration metadata and can be well below target_fps.
std_msgs/Header header
bool connected
float32 target_fps      # requested capture fps
float32 measured_fps    # actual fps from sensor metadata
int32 exposure_us       # current exposure time, microseconds
float32 analogue_gain
float32 red_gain
float32 green_gain      # software (display) green gain
float32 blue_gain
float32 colour_gain     # global multiplier on red/blue
float32 contrast
float32 saturation
float32 brightness
float32 sharpness
int32 width
int32 height
