# Live camera settings and the real frame rate, published on camera/state.
std_msgs/Header header
bool connected
float32 target_fps      # requested capture fps
float32 measured_fps    # delivered fps, from the camera server's encoder frame counter
int32 exposure_us       # current exposure time, microseconds
float32 analogue_gain
float32 red_gain
float32 green_gain      # kept for the contract; no effect
float32 blue_gain
float32 colour_gain     # global multiplier on red/blue
float32 contrast
float32 saturation
float32 brightness
float32 sharpness
int32 width
int32 height
