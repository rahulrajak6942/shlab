import pybullet as p
import numpy as np
import time

class RobotControl:
    """
    Control and Manipulation module for Franka Panda.
    Implements Inverse Kinematics and the Autonomous Grasp Pipeline.
    """
    def __init__(self, robot_id, ee_link=11):
        self.robot_id = robot_id
        self.ee_link = ee_link

    def move_to(self, target_pos, ori=None, tolerance=0.005):
        """Use Inverse Kinematics (IK) to reach desired 3D coordinates."""
        if ori is None:
            ori = p.getQuaternionFromEuler([np.pi, 0, 0])
        
        # Solving with high iterations for precision
        for _ in range(240):
            joint_poses = p.calculateInverseKinematics(self.robot_id, self.ee_link, target_pos, ori)
            for j in range(7):
                p.setJointMotorControl2(self.robot_id, j, p.POSITION_CONTROL, joint_poses[j], force=500)
            
            p.stepSimulation()
            current_ee = p.getLinkState(self.robot_id, self.ee_link)[4]
            if np.linalg.norm(np.array(target_pos) - np.array(current_ee)) < tolerance:
                break
            time.sleep(1/480.0)

    def control_gripper(self, dist, force=500):
        """Actuates the gripper fingers."""
        p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, dist, force=force)
        p.setJointMotorControl2(self.robot_id, 10, p.POSITION_CONTROL, dist, force=force)
        for _ in range(100): p.stepSimulation()

    def grasp_point_world(self, target_pos):
        """
        TECHNICAL REQUIREMENT: Grasp Pipeline Task 1.
        Sequence: Approach -> Descend -> Grasp -> Lift
        """
        # 1. Approach: Move above the object
        print("   [Pipeline] 1. Approach to pre-grasp pose")
        approach_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.12]
        self.move_to(approach_pos)
        
        # 2. Descend: Move to exact grasp coordinate
        print("   [Pipeline] 2. Descend to target centroid")
        self.move_to(target_pos)
        time.sleep(0.5)
        
        # 3. Grasp: Actuate fingers
        print("   [Pipeline] 3. Actuate Grasp")
        self.control_gripper(0.01) # Squeeze
        
        # 4. Lift: Raise to designated height
        print("   [Pipeline] 4. Executing Lift motion")
        lift_pos = [target_pos[0], target_pos[1], target_pos[2] + 0.2]
        self.move_to(lift_pos)
