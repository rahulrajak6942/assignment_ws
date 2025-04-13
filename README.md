# assignment_ws
Here's a draft for your `README.md`:

---

# ROS2 Navigation Assignment

## Overview

This project is part of an assignment to implement a modular navigation system for the Testbed-T1.0.0 robot using ROS2's Nav2 stack. The goal is to manually configure and launch the navigation components, including map loading, localization, and path planning.

## Repository Structure

```
ros_nav2_assignment/
├── testbed_description/
│   ├── launch/            # Launch the full base simulation
│   ├── meshes/
│   ├── rviz/              # RVIZ configuration files
│   └── urdf/              # URDF files for Testbed-T1.0.0
├── testbed_gazebo/
│   ├── worlds/            # Simulation world files
│   ├── launch/            # Launch files for Gazebo
│   └── models/            # Misc. Gazebo model files
├── testbed_bringup/
│   ├── launch/            # Launch file for bringing up the robot
│   └── maps/              # Predefined map of the test environment
└── testbed_navigation/
    ├── launch/            # Launch files for navigation
    ├── config/            # Configuration files for navigation
    └── rviz/              # RVIZ configuration files
```

## Setup Instructions

### 1. Environment Setup

- **ROS2 Humble**: Ensure ROS2 Humble is installed.
- **Gazebo**: Version 11.10.2 is compatible with ROS2 Humble.
- **Rviz**: For visualization.

### 2. Building the Workspace

1. Create your workspace:
   ```bash
   mkdir -p ~/assignment_ws/src
   cd ~/assignment_ws/src
   ```

2. Clone the repository:
   ```bash
   git clone https://github.com/rahulrajak6942/assignment_ws.git
   ```

3. Build the workspace:
   ```bash
   cd ~/assignment_ws
   colcon build
   source install/setup.bash
   ```

### 3. Launching the Simulation

1. Start the full simulation:
   ```bash
   ros2 launch testbed_bringup testbed_full_bringup.launch.py
   ```

2. Launch the map server:
   ```bash
   ros2 launch testbed_navigation map_loader.launch.py
   ```

3. Launch localization:
   ```bash
   ros2 launch testbed_navigation localization.launch.py
   ```

4. Launch navigation:
   ```bash
   ros2 launch testbed_navigation navigation.launch.py
   ```

### 4. RViz Configuration

- Set the "Fixed Frame" to "map".
- Add displays for Map, Global Costmap, Local Costmap, Path, and Global Path.
- Use "2D Pose Estimate" to set the initial position.
- Use "2D Goal Pose" to set navigation goals.

## Results

The robot successfully localizes and navigates within the simulated environment. Below is a screenshot of the setup in RViz:

![Simulation Screenshot](../Pictures/Screenshots/assign_sim.png)

## Challenges

- Configuring the costmap parameters to ensure proper visualization.
- Ensuring all transform frames are correctly set up.

## Conclusion

This project demonstrates the ability to manually configure and launch ROS2 navigation components, providing a modular and flexible navigation solution for the Testbed-T1.0.0 robot.

---

Feel free to modify the content as needed! Let me know if you need further assistance.
