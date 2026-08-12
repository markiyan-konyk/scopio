# Sample temperature controller (Wavelength Electronics TC10 LAB), polled by
# temperature_node. Carries what a control loop actually needs: the reading AND
# the setpoint it is chasing, the TEC drive, and the fault flags -- which is why
# this is not sensor_msgs/Temperature (see INTERFACES.md).
#
# temperature/setpoint/limits are in `units`; the node forces the units on
# connect so degrees are never ambiguous.
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
