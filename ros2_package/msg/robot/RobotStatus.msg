# Robot status and feedback message
bool connected                    # TCP connection status
bool moving                      # Whether robot is currently moving
bool enabled                     # Whether robot is enabled
float64[] current_joint_angles   # Current joint positions
float64[] current_position       # Current Cartesian position
float64 current_velocity         # Current movement velocity
string last_error               # Last error message
builtin_interfaces/Time timestamp # Status timestamp
