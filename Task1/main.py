"""
Robotics Engineer Assignment: Technical Task 1
Implementation of a Perception-driven Autonomous Pick-and-Place pipeline.
Robot: Franka Emika Panda
Engine: PyBullet
"""

from modules.environment import RobotEnv
from modules.perception import CameraSystem
from modules.manipulator import RobotControl
import pybullet as p
import numpy as np
import time
import argparse

def classify_object(color_rgb):
    """Heuristic Classification for Task 1 Requirements."""
    r, g, b = color_rgb[:3]
    if r > 0.8 and g < 0.2: return "RED CUBE"
    if g > 0.8 and r < 0.2: return "GREEN CUBE"
    if b > 0.8 and r < 0.2: return "BLUE CUBE"
    return "UNKNOWN OBJECT"

def main(gui=True):
    # 1. Initialization
    env = RobotEnv(gui=gui)
    camera = CameraSystem(env.robot_id)
    controller = RobotControl(env.robot_id)
    
    print("\n" + "="*60)
    print("TASK 1: AUTONOMOUS PICK-AND-PLACE DEMONSTRATION")
    print("="*60)
    
    # 2. Scene Setup: Spawn multiple colored objects
    # Strategy: Spawn 5 cubes, but we will ONLY pick the RED one.
    target_colors = [
        [1, 0, 0, 1],   # Red (Target)
        [0, 1, 0, 1],   # Green (Ignore)
        [0, 0, 1, 1],   # Blue (Ignore)
        [1, 1, 0, 1],   # Yellow (Ignore)
        [1, 0, 1, 1]    # Purple (Ignore)
    ]
    cubes = []
    print(f"\n[Scene Setup] Spawning {len(target_colors)} objects on the table...")
    for color in target_colors:
        cubes.append(env.spawn_cube(color=color))
    
    # 3. Filtering Logic: Target only RED
    red_cubes = [c for c in cubes if classify_object(c[1]) == "RED CUBE"]
    ignored_cubes = [c for c in cubes if classify_object(c[1]) != "RED CUBE"]
    
    print(f"\n[Strategy] Found {len(red_cubes)} RED cube(s) to pick.")
    print(f"[Strategy] Ignoring {len(ignored_cubes)} other objects as per requirement.")

    # Iterate ONLY through Red objects
    for i, (cube_id, color) in enumerate(red_cubes):
        obj_type = classify_object(color)
        print(f"\n>>> [MISSION] Targeting: {obj_type} (ID: {cube_id})")
        
        # 3. Vision Phase 1: Global Localization (Overhead Camera)
        ov_view = camera.get_overhead_view()
        _, _, seg = camera.capture(ov_view)
        mask = (seg == cube_id)
        
        if np.any(mask):
            coords = np.argwhere(mask)
            v, u = coords.mean(axis=0).astype(int)
            approx_world = camera.back_project(u, v, 0.9, ov_view) 
        else:
            print("Target not in view!")
            continue

        # 4. Vision Phase 2: High Precision Detection (Wrist Camera)
        scan_pos = [approx_world[0], approx_world[1], 0.60]
        controller.move_to(scan_pos)
        
        wr_view = camera.get_wrist_view()
        _, d_buf, seg = camera.capture(wr_view)
        
        mask = (seg == cube_id)
        if np.any(mask):
            coords = np.argwhere(mask)
            v, u = coords.mean(axis=0).astype(int)
            d_val = d_buf[v, u]
            surface_pos = camera.back_project(u, v, d_val, wr_view)
            print(f"   [Perception] Exact Red Cube Location: {np.round(surface_pos, 3)}")
            
            if gui:
                p.addUserDebugText("TARGET: RED", surface_pos, textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=10)
        else:
            print("Precision scan failed.")
            continue

        # 5. Control Phase: Autonomous Grasp Pipeline
        controller.control_gripper(0.04) 
        grasp_target_z = surface_pos[2] - 0.025 
        pick_pos = [surface_pos[0], surface_pos[1], grasp_target_z]
        
        controller.grasp_point_world(pick_pos)
        
        # 6. Place Phase
        place_pos = [0.4, -0.4, 0.42] # Fixed "Red Bin" location
        controller.move_to([place_pos[0], place_pos[1], 0.6]) 
        controller.move_to(place_pos)
        controller.control_gripper(0.04) 
        controller.move_to([place_pos[0], place_pos[1], 0.6]) 

    print("\n" + "="*60)
    print("TASK COMPLETED: Red cube(s) isolated and moved.")
    print("="*60)
    
    if gui:
        while True:
            p.stepSimulation()
            time.sleep(1/240.0)
    else:
        p.disconnect()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    
    try:
        main(gui=not args.headless)
    except KeyboardInterrupt:
        p.disconnect()
