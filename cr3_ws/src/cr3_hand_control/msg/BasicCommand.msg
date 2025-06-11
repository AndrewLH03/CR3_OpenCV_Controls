# Basic command message for CR3 robot control
# Command types: "enable", "disable", "clear_error", "movj", "movl", "get_pose"
string command_type

# Position array [x, y, z, rx, ry, rz] (6 DOF)
float64[6] position

# Movement speed (0.0 to 1.0)
float64 speed

# Additional command parameters
string[] parameters

# Timestamp
builtin_interfaces/Time timestamp
