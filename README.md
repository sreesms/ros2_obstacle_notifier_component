# ROS 2 Obstacle Notifier Component

A simple ROS 2 component node that listens to LaserScan data and notifies when an obstacle is detected within a specified distance.  
This project demonstrates the use of ROS 2 components and dynamic node loading using component containers.

---

## Features

- Implemented as a **component node**
- Subscribes to `/scan` topic (`sensor_msgs/msg/LaserScan`)
- Warns when obstacles are within a threshold distance
- Can be loaded/unloaded **at runtime** using `component_container`

---

## Code Integration

### In the C++ file
At the end of your component node source:

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_notifier::ObstacleNotifierComponent)
```
---

## CMake Configuration
```xml
rclcpp_components_register_nodes(obstacle_notifier_component
  "obstacle_notifier::ObstacleNotifierComponent"
)

install(TARGETS obstacle_notifier_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
```
---
## How to Run
### 1. Start Component Container
```bash
ros2 run rclcpp_components component_container
```
You should see the node /ComponentManager.
Check with 
```bash
ros2 node list
```

### 2. Load the Component
```bash
ros2 component load /ComponentManager obstacle_notifier_component obstacle_notifier::ObstacleNotifierComponent
```
Expected output from:
```bash
ros2 node list

/ComponentManager
/obstacle_notifier
```
### 3. Test with LaserScan
Make sure a LaserScan topic (/scan) is publishing (from simulation or real robot).
The component will log a warning when obstacles are closer than the threshold.

### 4. Unload the Component

First, list components:
```bash
ros2 component list /ComponentManager
```
Then unload using the instance ID (e.g., 1):

```bash
ros2 component unload /ComponentManager 1
```
---
## Why Use Components?

  - Load/unload nodes without restarting ROS 2

  - Share resources (executors, threads) efficiently

  - Useful for modular and runtime-flexible systems