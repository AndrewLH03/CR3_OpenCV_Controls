# Debug information for pose recognition
std_msgs/Header header

# Performance debugging
float32 camera_fps
float32 mediapipe_fps  
float32 total_processing_time
float32 pose_processing_time
float32 hand_processing_time

# Detection quality metrics
int32 total_frames_processed
int32 frames_with_pose
int32 frames_with_hand
float32 pose_detection_rate
float32 hand_detection_rate

# System health
string camera_status
string mediapipe_status
string[] error_messages
string[] warning_messages
