# Status of the galvo/laser AWG, which the galvo_node drives as a raw VISA/SCPI
# passthrough. The node does NOT interpret command meaning (no volts/pixels) --
# laser geometry lives in client code. This just reports liveness and the last
# command relayed, so clients can confirm the link.
std_msgs/Header header
bool connected
string idn            # *IDN? response of the connected instrument
string last_command   # last SCPI string relayed to the instrument
string last_error     # last error (empty if none)
