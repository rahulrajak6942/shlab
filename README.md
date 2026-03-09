# SHLab: Robotic Arm Motion Planning in Cluttered Environments

![ROS 2](https://img.shields.io/badge/ROS-Humble-blue)
![MoveIt 2](https://img.shields.io/badge/MoveIt-2-orange)
![Gazebo](https://img.shields.io/badge/Gazebo-Ignition-blue)

## Project Demonstration

![Project Video](plannar.webm)

## Project Overview

This project implements a complete pipeline for a 6-DOF robotic arm to navigate through a "forest" of obstacles in a simulated environment. The core of the project is a custom **Hybrid APF-RRT Planner** which combines the global exploration capabilities of Rapidly-exploring Random Trees (RRT) with the local obstacle avoidance and goal attraction of Artificial Potential Fields (APF).

### Key Features
*   **Custom Planner**: Integration of RRT with APF gradients to handle tight spaces and complex obstacle configurations.
*   **Environment Sync**: Real-time synchronization between the Gazebo world and the MoveIt planning scene for high-fidelity collision checking.
*   **Robust Executor**: Multi-threaded ROS 2 node capable of calculating Inverse Kinematics (IK), planning paths, and executing trajectories while monitoring for collisions.
*   **Complex Simulation**: A custom Gazebo world filled with cylinders, spheres, and boxes designed to test the limits of motion planning algorithms.

## Repository Structure

*   **`shlab_executor`**: Contains the Hybrid Planner implementation (C++) and the main execution node.
*   **`shlab_control`**: Configuration for Gazebo simulation, hardware controllers, and the world SDF.
*   **`shlab_description`**: URDF/XACRO models for the robot arm and end-effector.
*   **`shlab_moveit_config`**: Configuration files for MoveIt 2 (SRDF, kinematics, planning pipeline).

---

## Installation & Setup

### Prerequisites
*   Ubuntu 22.04
*   ROS 2 Humble
*   MoveIt 2
*   Ignition Gazebo (Citadel/Fortress)

### Build
```bash
# Clone the repository into your workspace src folder
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## How to Run

### 1. Launch the Simulation
This starts Gazebo, MoveIt, and the Executor node:
```bash
ros2 launch shlab_executor run.launch.py
```

### 2. Request a Goal
Open a new terminal and send a Cartesian target `[X, Y, Z]` to the robot:
```bash
ros2 service call /executor shlab_description/srv/Goal "{position: [0.4, -0.1, 0.5]}"
```

## How it Works

1.  **Inverse Kinematics**: When a target XYZ is received, the executor uses MoveIt's KDL solver to find valid joint angles.
2.  **Collision Validation**: The solver checks if the goal pose or any point along the path collides with the "Forest of Obstacles."
3.  **Hybrid Planning**:
    *   **RRT**: Randomly samples the space to find a route.
    *   **APF Gradient**: Instead of simple straight lines between nodes, the planner "steers" using potential fields—attracted to the goal but repelled by obstacles.
4.  **Trajectory Execution**: The resulting path is smoothed and sent to the `Joint Trajectory Controller` in Gazebo.

---

## Troubleshooting
*   **Goal Failed**: Check if the target point is inside a cylinder or box. Move the `z` value higher to clear obstacles.
*   **Sim Time**: If the robot isn't moving, ensure `use_sim_time:=true` is set (our launch files handle this by default).
