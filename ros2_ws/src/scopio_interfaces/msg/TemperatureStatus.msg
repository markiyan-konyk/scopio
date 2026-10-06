# TC10 LAB state for a control loop (reading, setpoint, drive, faults), polled; temperatures are in `units`.
std_msgs/Header header
bool connected
string idn            # *IDN? of the connected controller
float32 temperature   # actual, at the selected sensor (NaN when disconnected)
float32 setpoint
float32 current       # TEC current, Amps (signed: + heats, - cools)
float32 voltage       # TEC voltage, Volts
bool output           # TEC output current enabled (nothing moves until true)
bool in_tolerance     # inside the tolerance window (see set_tolerance)
string units          # "C" | "K" | "F" | "raw"
uint16 condition      # raw TEC:COND? bitfield
string[] faults       # decoded fault bits, e.g. ["sensor_open"]; empty == healthy
string last_error     # last error (empty if none)
