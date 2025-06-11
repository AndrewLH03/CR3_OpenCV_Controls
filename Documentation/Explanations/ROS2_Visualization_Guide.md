# ROS2 Visualization and Troubleshooting Guide

*A practical guide for understanding what you see in ROS2 visualization tools*

## 🎯 Understanding rqt_graph: What You See and Why

### Current State: Single Node, No Connections

When you run `rqt_graph` and see only the `/basic_robot_controller` node with no arrows, this is completely normal for Phase 2. Here's why:

#### What You're Seeing
```
┌─────────────────────────┐
│  /basic_robot_controller │  ← Single oval, no connections
└─────────────────────────┘
```

#### Why No Connections Appear
1. **Node exists but isn't communicating**: Your C++ code creates a node but doesn't create any publishers or subscribers yet
2. **Topics defined but not used**: You've defined message types but the code doesn't use them
3. **Services available but not active**: Service definitions exist but no active connections

### What "No Arrows" Means

**Arrows represent active communication**:
- **Outgoing arrow**: Node publishes to a topic
- **Incoming arrow**: Node subscribes to a topic
- **Bidirectional**: Node both publishes and subscribes

**No arrows = No active publishers or subscribers**

## 🔧 Making Connections Visible

To see connections in rqt_graph, you need active communication. Here are ways to test:

### Method 1: Run Multiple Nodes

**Terminal 1:**
```bash
cd /home/andrewlh/CR3_OpenCV_Controls/ros2_package
source install/setup.bash
ros2 run cr3_hand_control basic_robot_controller
```

**Terminal 2:**
```bash
source /home/andrewlh/CR3_OpenCV_Controls/ros2_package/install/setup.bash
ros2 run cr3_hand_control diagnostics_monitor.py
```

**Terminal 3:**
```bash
source /home/andrewlh/CR3_OpenCV_Controls/ros2_package/install/setup.bash
ros2 run rqt_graph rqt_graph
```

Now you should see both nodes in the graph.

### Method 2: Use Launch Files

**Single command to start multiple nodes:**
```bash
cd /home/andrewlh/CR3_OpenCV_Controls/ros2_package
source install/setup.bash
ros2 launch cr3_hand_control test_nodes.launch.py
```

This starts both `basic_robot_controller` and `diagnostics_monitor` simultaneously.

### Method 3: Manually Publish/Subscribe

**Create artificial connections for testing:**

**Terminal 1:** Start your node
```bash
ros2 run cr3_hand_control basic_robot_controller
```

**Terminal 2:** Manually publish to a topic
```bash
ros2 topic pub /test_topic std_msgs/msg/String "data: 'Hello Robot'"
```

**Terminal 3:** Subscribe to topics
```bash
ros2 topic echo /test_topic
```

**Terminal 4:** View the graph
```bash
ros2 run rqt_graph rqt_graph
```

Now you'll see connections between the manual publisher/subscriber and any topics.

## 🎨 rqt_graph Display Options

### View Modes (Top of rqt_graph window)

1. **Nodes only** ← (What you currently see)
   - Shows only nodes as ovals
   - No topic information displayed

2. **Nodes/Topics (all)**
   - Shows nodes as ovals
   - Shows topics as rectangles
   - Shows all defined topics, even unused ones

3. **Nodes/Topics (active)** ← (Most useful for debugging)
   - Shows only active communication
   - Topics appear only if someone publishes/subscribes

### Display Settings

**Checkboxes to toggle:**
- ✅ **Group namespaces**: Groups related nodes together
- ✅ **Hide debug**: Removes ROS2 internal nodes (cleaner view)
- ✅ **Unreachable**: Shows orphaned nodes

### What Different Shapes Mean

- **Ovals**: ROS2 nodes (your programs)
- **Rectangles**: Topics (communication channels)
- **Diamonds**: Services (rare in basic view)
- **Arrows**: Data flow direction

## 🔍 Command Line Alternatives

When rqt_graph isn't showing what you expect, use these commands:

### Node Information
```bash
# List all nodes
ros2 node list

# Detailed info about a specific node
ros2 node info /basic_robot_controller
```

### Topic Information
```bash
# List all topics
ros2 topic list

# Show topic types
ros2 topic list -t

# Show who publishes/subscribes to a topic
ros2 topic info /robot_status

# Monitor messages in real-time
ros2 topic echo /robot_status
```

### Service Information
```bash
# List all services
ros2 service list

# Show service types
ros2 service list -t

# Get info about a service
ros2 service type /emergency_stop
```

## 🐛 Common Issues and Solutions

### Issue: "No nodes showing"
**Cause**: No ROS2 nodes are running
**Solution**: 
```bash
# Check if any nodes are running
ros2 node list
# If empty, start your nodes first
```

### Issue: "Nodes show but no topics"
**Cause**: Nodes aren't publishing or subscribing
**Solution**: Check if your node code actually creates publishers/subscribers

### Issue: "rqt_graph shows too many nodes"
**Cause**: ROS2 internal nodes are visible
**Solution**: Check "Hide debug" in rqt_graph settings

### Issue: "Graph keeps changing"
**Cause**: Nodes starting/stopping, or intermittent connections
**Solution**: Use "Nodes/Topics (active)" view to see only stable connections

## 📊 System Evolution Through All 10 Phases

Watch how your ROS2 system grows from a single node to a complex, interconnected robot control system:

### Phase 1: Basic TCP Connection (3-4 days)
```
┌─────────────────────────┐
│  /basic_robot_controller │  ← Single node, TCP connection to robot
└─────────────────────────┘
            │
            │ TCP/IP
            ↓
   ┌─────────────────┐
   │  Physical CR3   │
   │  Robot          │
   └─────────────────┘
```
**Key Features**: Basic TCP socket communication, simple movement commands

### Phase 2: ROS2 Infrastructure (2-3 days) ← **YOU ARE HERE**
```
┌─────────────────────────┐
│  /basic_robot_controller │  ← Infrastructure built, minimal communication
└─────────────────────────┘

┌─────────────────────────┐
│  /parameter_manager     │  ← Parameter handling ready
└─────────────────────────┘

┌─────────────────────────┐
│  /diagnostics_monitor   │  ← System health monitoring
└─────────────────────────┘
```
**Key Features**: Message/service definitions, parameter system, build infrastructure

### Phase 3: Workspace Safety & Coordinate Systems (2 days)
```
┌─────────────────────────┐    /robot_status    ┌─────────────────────────┐
│  /basic_robot_controller │ ─────────────────→ │  /safety_monitor        │
│                         │                     │                         │
│                         │ ←────────────────── │                         │
└─────────────────────────┘    /emergency_stop  └─────────────────────────┘
            │                                              │
            │ /coordinate_transforms                       │ /safety_alerts
            ↓                                              ↓
   ┌─────────────────┐                            ┌─────────────────┐
   │ /tf2_broadcaster│                            │ /collision_     │
   │                 │                            │  detector       │
   └─────────────────┘                            └─────────────────┘
```
**Key Features**: Workspace boundaries, coordinate transformations, collision detection

### Phase 4: Hand Tracking Foundation (4-5 days)
```
┌─────────────────┐    /hand_position    ┌─────────────────────────┐
│ /hand_tracker   │ ───────────────────→ │  /basic_robot_controller │
│ (OpenCV+        │                      │                         │
│  MediaPipe)     │                      │                         │
└─────────────────┘                      │                         │
            │                            │                         │
            │ /hand_confidence           │                         │
            ↓                            │                         │
┌─────────────────┐                      │                         │
│ /hand_validator │ ──────────────────── │                         │
└─────────────────┘   /validated_pose    │                         │
                                         │                         │
┌─────────────────┐ ←─────────────────── │                         │
│ /safety_monitor │    /safety_status    └─────────────────────────┘
└─────────────────┘
```
**Key Features**: Real-time hand detection, pose validation, gesture recognition

### Phase 5: Advanced Robot Control & Motion Planning (4-5 days)
```
                    /hand_position
┌─────────────────┐ ────────────────→ ┌─────────────────────────┐
│ /hand_tracker   │                   │ /motion_planner         │
└─────────────────┘                   │                         │
                                      │                         │
┌─────────────────┐ ←──────────────── │                         │
│ /trajectory_    │  /planned_path    │                         │
│  executor       │                   │                         │
└─────────────────┘                   │                         │
            │                         │                         │
            │ /joint_commands         │                         │
            ↓                         │                         │
┌─────────────────┐ ←──────────────── │                         │
│/robot_controller│  /motion_goal     └─────────────────────────┘
└─────────────────┘                            │
            │                                  │ /robot_feedback
            │ TCP/IP                           ↓
            ↓                         ┌─────────────────┐
   ┌─────────────────┐                │ /safety_monitor │
   │  Physical CR3   │ ──────────────→│                 │
   │  Robot          │  /robot_state  └─────────────────┘
   └─────────────────┘
```
**Key Features**: Advanced motion planning, trajectory optimization, smooth control

### Phase 6: Communication Bridge & Data Flow (3-4 days)
```
                /raw_hand_data
┌─────────────────┐ ────────────────→ ┌─────────────────────────┐
│ /camera_node    │                   │ /data_bridge            │
└─────────────────┘                   │                         │
                                      │                         │
┌─────────────────┐ ←──────────────── │                         │
│ /hand_processor │  /processed_data  │                         │
└─────────────────┘                   │                         │
            │                         │                         │
            │ /filtered_position      │                         │
            ↓                         │                         │
┌─────────────────┐ ←──────────────── │                         │
│ /coordinate_    │  /sync_signal     └─────────────────────────┘
│   transformer   │                          │
└─────────────────┘                          │ /unified_commands
            │                                ↓
            │ /robot_coordinates   ┌─────────────────┐
            ↓                      │ /motion_planner │
┌─────────────────┐ ──────────────→│                 │
│/robot_controller│                └─────────────────┘
└─────────────────┘
```
**Key Features**: Data synchronization, coordinate bridging, unified command interface

### Phase 7: System Integration & Testing (4-5 days)
```
┌─────────────────┐    /camera_feed    ┌─────────────────────────┐
│ /camera_node    │ ─────────────────→ │ /integration_manager    │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐    /hand_pose      │                         │
│ /hand_tracker   │ ─────────────────→ │                         │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐    /safety_check   │                         │
│ /safety_monitor │ ─────────────────→ │                         │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐ ←───────────────── │                         │
│/robot_controller│   /integrated_cmd  │                         │
└─────────────────┘                    │                         │
            │                          │                         │
            │                          │                         │
            ↓                          │                         │
┌─────────────────┐ ─────────────────→ │                         │
│ /system_monitor │   /performance     └─────────────────────────┘
└─────────────────┘                            │
                                               │ /test_results
                                               ↓
                                      ┌─────────────────┐
                                      │ /test_validator │
                                      └─────────────────┘
```
**Key Features**: End-to-end integration, automated testing, performance validation

### Phase 8: Performance Optimization & Real-time Tuning (4-5 days)
```
┌─────────────────┐    /metrics        ┌─────────────────────────┐
│ /performance_   │ ─────────────────→ │ /optimization_engine    │
│  monitor        │                    │                         │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐    /latency        │                         │
│ /latency_       │ ─────────────────→ │                         │
│  analyzer       │                    │                         │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐ ←───────────────── │                         │
│ /dynamic_tuner  │   /tuning_params   │                         │
└─────────────────┘                    │                         │
            │                          │                         │
            │ /optimized_settings      │                         │
            ↓                          │                         │
┌─────────────────┐ ←───────────────── │                         │
│ /motion_planner │   /realtime_adjust └─────────────────────────┘
│ (optimized)     │                            │
└─────────────────┘                            │ /performance_report
            │                                  ↓
            │ /smooth_commands         ┌─────────────────┐
            ↓                          │ /analytics_node │
┌─────────────────┐                    └─────────────────┘
│/robot_controller│
│ (high-perf)     │
└─────────────────┘
```
**Key Features**: Real-time optimization, latency minimization, adaptive tuning

### Phase 9: User Interface & Visualization (4-5 days)
```
┌─────────────────┐    /ui_commands    ┌─────────────────────────┐
│ /web_interface  │ ─────────────────→ │ /ui_manager             │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐    /user_input     │                         │
│ /control_panel  │ ─────────────────→ │                         │
└─────────────────┘                    │                         │
                                       │                         │
┌─────────────────┐ ←───────────────── │                         │
│ /3d_visualizer  │   /robot_state     │                         │
└─────────────────┘                    │                         │
            ↑                          │                         │
            │ /hand_overlay            │                         │
┌─────────────────┐                    │                         │
│ /hand_tracker   │ ─────────────────→ │                         │
└─────────────────┘   /visual_feed     │                         │
                                       │                         │
┌─────────────────┐ ←───────────────── │                         │
│ /status_display │   /system_status   └─────────────────────────┘
└─────────────────┘                            │
            ↑                                  │ /dashboard_data
            │                                  ↓
┌─────────────────┐                   ┌─────────────────┐
│ /alert_system   │ ←──────────────── │ /data_aggregator│
└─────────────────┘   /alerts         └─────────────────┘
```
**Key Features**: Web dashboard, 3D visualization, real-time monitoring, user controls

### Phase 10: Documentation & Deployment (4-5 days)
```
┌─────────────────┐    /deployment    ┌─────────────────────────┐
│ /config_manager │ ────────────────→ │ /deployment_orchestrator│
└─────────────────┘                   │                         │
                                      │                         │
┌─────────────────┐    /health_check  │                         │
│ /health_monitor │ ────────────────→ │                         │
└─────────────────┘                   │                         │
                                      │                         │
┌─────────────────┐ ←──────────────── │                         │
│ /backup_manager │  /backup_trigger  │                         │
└─────────────────┘                   │                         │
            │                         │                         │
            │ /system_backup          │                         │
            ↓                         │                         │
┌─────────────────┐                   │                         │
│ /documentation_ │ ────────────────→ │                         │
│  generator      │   /auto_docs      │                         │
└─────────────────┘                   │                         │
                                      │                         │
┌─────────────────┐ ←──────────────── │                         │
│ /update_manager │  /version_control └─────────────────────────┘
└─────────────────┘                            │
            │                                  │ /deployment_status
            │ /update_signal                   ↓
            ↓                         ┌─────────────────┐
┌─────────────────┐                   │ /monitoring_    │
│ ALL SYSTEM      │ ←──────────────── │  dashboard      │
│ NODES           │   /system_control └─────────────────┘
│ (production)    │
└─────────────────┘
```
**Key Features**: Automated deployment, system monitoring, backup management, documentation

## 🎯 Complexity Growth Analysis

### Node Count Evolution
- **Phase 1-2**: 1-3 nodes (Basic infrastructure)
- **Phase 3-4**: 4-6 nodes (Core functionality)
- **Phase 5-7**: 8-12 nodes (Advanced features)
- **Phase 8-10**: 15+ nodes (Full production system)

### Communication Complexity
- **Early Phases**: Simple point-to-point communication
- **Mid Phases**: Hub-and-spoke patterns emerge
- **Late Phases**: Complex mesh networks with multiple data flows

### Key Observation Points
- **Phase 2→3**: First real inter-node communication appears
- **Phase 4→5**: System becomes genuinely distributed
- **Phase 7→8**: Performance becomes critical factor
- **Phase 9→10**: User experience and deployment focus

## 🎯 Testing Checklist

To verify your ROS2 system is working correctly:

- [ ] Can start individual nodes without errors
- [ ] `ros2 node list` shows your nodes
- [ ] `ros2 topic list` shows available topics
- [ ] rqt_graph displays your nodes (even without connections)
- [ ] Can manually publish/subscribe to test topics
- [ ] Launch files start multiple nodes successfully

If all these work, your Phase 2 infrastructure is solid and ready for Phase 3 development!

---

*This guide helps interpret what you see in ROS2 visualization tools during different development phases*
