# Pose tracking status information
std_msgs/Header header

# Detection status
bool tracking_active
bool pose_detected
bool hand_detected
float32 pose_confidence
float32 hand_confidence

# Performance metrics
float32 fps
int32 frame_count
float32 processing_time_ms

# Robot integration status
bool robot_enabled
bool robot_connected
string status_message
