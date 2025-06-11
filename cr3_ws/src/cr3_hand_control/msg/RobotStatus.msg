# Robot status message for CR3 robot feedback
# Connection status
bool connected
string connection_message

# Current robot position [x, y, z, rx, ry, rz] (6 DOF)
float64[6] current_position

# Robot state
bool enabled
bool error_state
string error_message

# Network configuration
string robot_ip
int32 dashboard_port

# Timestamp
builtin_interfaces/Time timestamp
