import pybullet as p
import pybullet_data
import numpy as np
import random

class RobotEnv:
    def __init__(self, gui=True):
        self.gui = gui
        connect_mode = p.GUI if gui else p.DIRECT
        self.client = p.connect(connect_mode)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240.0)
        
        p.loadURDF("plane.urdf")
        
        # Table with surface friction
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.2])
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.2], rgbaColor=[0.8, 0.8, 0.8, 1])
        self.table_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=[0.5, 0, 0.2])
        p.changeDynamics(self.table_id, -1, lateralFriction=1.0)

        # Load Panda
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
        # Set high friction for fingers
        p.changeDynamics(self.robot_id, 9, lateralFriction=2.0, spinningFriction=1.0)
        p.changeDynamics(self.robot_id, 10, lateralFriction=2.0, spinningFriction=1.0)
        
        self.reset_home()
        
    def reset_home(self):
        home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        for i in range(7):
            p.resetJointState(self.robot_id, i, home[i])
        p.resetJointState(self.robot_id, 9, 0.04)
        p.resetJointState(self.robot_id, 10, 0.04)

    def spawn_cube(self, color=None):
        size = 0.025 # 5cm cube
        x, y = random.uniform(0.4, 0.6), random.uniform(-0.15, 0.15)
        if color is None:
            color = [random.random(), random.random(), random.random(), 1]
        
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=color)
        cube_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=[x, y, 0.45])
        p.changeDynamics(cube_id, -1, lateralFriction=1.0, rollingFriction=0.01)
        # Settle
        for _ in range(50): p.stepSimulation()
        return cube_id, color
