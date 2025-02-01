# Robotics_Project

**Project Overview:** Lane Painting Path Robot
**Project Structure**
The project is a ROS 2 (Robot Operating System 2) based robotics application that simulates a robot capable of following a certain path and painting a yellow line along its path.

**Key Directories and Files**

1. **/src/:** Source code directory
   - **robot_controller_node.cpp:** Main robot control logic
   - **lane_detector_node.cpp:** Lane detection functionality (not fully explored)
2. **/urdf/:** Robot description
   - **robot.urdf.xacro:** Robot's Unified Robot Description Format (URDF) file
   - Defines robot's physical characteristics:
     - Base link dimensions
     - Wheel specifications
     - Gazebo simulation plugins
3. **/launch/:** Launch configuration
   - robot_launch.py: ROS 2 launch file for starting the robot simulation
4. **/config/:** Configuration files
   - gui.config: Potential GUI configuration
5. **CMakeLists.txt:** Build configuration for the project

**Robot Specifications**

- **Type:** Differential drive robot
- **Base Dimensions:**
  - Length: 0.8m
  - Width: 0.6m
  - Height: 0.3m
- **Wheels:**
  - Radius: 0.15m
  - Separation: 0.5m
- **Color:** Metallic blue base

**Simulation Features**

- Uses Gazebo Harmonic (gz-sim) for physics simulation
- Differential drive system plugin
- New paint marker system plugin to create a red paint trail

**Key Functionalities**

1. **Lane Following**
   - Subscribes to odometry and command velocity topics
   - Implements smooth acceleration and deceleration
   - Can receive and process movement commands
2. **Paint Marking**
   - Newly added feature to paint a red line while moving
   - Configurable paint color, interval, and width
   - Uses gz-sim-paint-marker-system plugin

**Potential Improvements**

1. Enhance lane detection algorithm
2. Add more sophisticated paint marking controls
3. Implement more advanced robot control logic
4. Add sensor simulation (e.g., camera, lidar)

**Dependencies**

- ROS 2 (Jazzy)
- Gazebo Harmonic
- OpenCV
- TF2 transformation library

build the project with the following command:

```
colcon build --packages-select lane_painting_path_robot

```

```
source install/setup.bash
```

run the project with the following command:

```
ros2 launch lane_painting_path_robot lane_painting_path_robot.launch.py
```
