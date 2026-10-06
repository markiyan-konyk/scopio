# Spatial calibration, latched on `calibration`. Converting um_per_px across sensor modes: scopio_microscope/README.md.
std_msgs/Header header
bool has_um_per_px
float64 um_per_px        # micrometres per pixel of a frame um_per_px_width pixels wide
int32 um_per_px_width    # frame width the scale was measured at (0 = unknown)
int32 um_per_px_window   # sensor pixels that frame spanned; scale tracks window/width, not width (0 = unknown)
float64 steps_per_um_x   # Sangaboard steps per micrometre, per axis
float64 steps_per_um_y
float64 steps_per_um_z
