# Coordinate transformation message for camera-robot coordination
# Used to communicate coordinate frame transformations

string source_frame      # Source coordinate frame (e.g., "camera_link")
string target_frame      # Target coordinate frame (e.g., "base_link")
geometry_msgs/Transform transform    # The transformation matrix
float64 accuracy         # Transformation accuracy (mm)
bool is_valid           # Whether transformation is currently valid
builtin_interfaces/Time timestamp
