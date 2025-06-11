# Hand position tracking message
# Contains 3D position and orientation data from hand tracking

geometry_msgs/Point position          # 3D position in workspace coordinates
geometry_msgs/Quaternion orientation  # Hand orientation
float32 confidence                     # Detection confidence (0.0-1.0)
bool is_valid                          # Whether the detection is valid
builtin_interfaces/Time timestamp      # Detection timestamp
