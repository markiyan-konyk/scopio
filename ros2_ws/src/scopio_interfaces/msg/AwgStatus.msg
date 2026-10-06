# Liveness of the galvo AWG link, published on awg/status; the node does not interpret commands.
std_msgs/Header header
bool connected
string idn            # *IDN? response of the connected instrument
string last_command   # last SCPI string relayed to the instrument
string last_error     # last error (empty if none)
