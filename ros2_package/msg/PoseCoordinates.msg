# Custom message for pose tracking coordinates (shoulder + wrist)
std_msgs/Header header

# Shoulder position
geometry_msgs/Point shoulder_position
float32 shoulder_confidence

# Wrist position
geometry_msgs/Point wrist_position
float32 wrist_confidence

# Detection status
bool pose_detected
bool hand_detected
string tracked_hand  # "Right" or "Left"
