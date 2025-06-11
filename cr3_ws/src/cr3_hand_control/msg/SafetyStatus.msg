# Safety status message
# Contains comprehensive safety system status

uint8 SAFE = 0
uint8 WARNING = 1
uint8 DANGER = 2
uint8 EMERGENCY = 3

uint8 safety_level                      # Current safety level
string[] active_warnings               # List of active warnings
string[] safety_zones_violated         # Violated safety zones
bool emergency_stop_active             # Emergency stop status
float32 workspace_proximity            # Distance to workspace boundary (meters)
builtin_interfaces/Time last_update    # Last safety check timestamp
