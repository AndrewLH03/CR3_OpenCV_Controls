# Safety system status message
bool workspace_safe              # Whether robot is within safe workspace
bool collision_detected          # Whether collision is detected
bool emergency_stop_active       # Whether emergency stop is engaged
float64 distance_to_boundary     # Distance to nearest workspace boundary
float64 distance_to_obstacle     # Distance to nearest obstacle
string[] active_warnings         # List of active safety warnings
uint8 safety_level              # 0=Safe, 1=Warning, 2=Danger, 3=Emergency
builtin_interfaces/Time timestamp # Safety status timestamp
