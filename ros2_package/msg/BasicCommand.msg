# Basic movement command message
string command_type    # "MovJ", "MovL", "Stop", "Home" 
float64[] joint_angles # Joint angles in degrees (6 values for CR3)
float64[] position     # Cartesian position [x, y, z, rx, ry, rz]
float64 velocity       # Movement velocity (0.0-100.0)
bool relative          # Whether movement is relative to current position
