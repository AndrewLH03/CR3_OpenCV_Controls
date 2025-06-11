# Safety alert message for the CR3 robot system
# Used to communicate safety-related events and warnings

uint8 level              # 0=Info, 1=Warning, 2=Danger, 3=Emergency
string alert_type        # "boundary", "collision", "speed", "system"
string description       # Human-readable alert description
geometry_msgs/Point location    # Location of safety concern
float64 distance_to_boundary    # Distance to nearest boundary (mm)
float64 recommended_speed       # Recommended speed for safety (mm/s)
builtin_interfaces/Time timestamp
