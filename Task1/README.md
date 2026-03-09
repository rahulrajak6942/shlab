# Task 1: Perception & Control

This repository contains an autonomous pick-and-place pipeline implemented in PyBullet for the Franka Emika Panda robot.

## Features
- **Simulation**: PyBullet environment with custom table and random cube generation.
- **Perception**: Multi-camera system (Wrist and Overhead) with 3D Back-Projection from pixel coordinates to world space.
- **Control**: IK-based motion planning with a state-machine grasp pipeline (Approach, Descend, Grasp, Lift).

## Installation
Ensure you have the required dependencies installed:
```bash
pip install pybullet numpy
```

## Usage
Run the main script to start the simulation:
```bash
python3 main.py
```

## Technical Implementation
- **Back-Projection**: Implements the transformation $P_{world} = T_{camera\_world} \cdot K^{-1} \cdot P_{pixel} \cdot d$ to map 2D vision data into robot task space.
- **Grasp Pipeline**: Automated via the `grasp_point_world()` function.
