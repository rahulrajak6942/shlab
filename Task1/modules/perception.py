import pybullet as p
import numpy as np

class CameraSystem:
    """
    Advanced Perception System for Task 1:
    Handles Overhead and Wrist-mounted camera streams, intrinsic/extrinsic calibration,
    and 3D back-projection logic.
    """
    def __init__(self, robot_id, width=640, height=480):
        self.robot_id = robot_id
        self.width = width
        self.height = height
        self.fov = 90
        self.near, self.far = 0.01, 5.0
        
        # 1. Coordinate Transformation: Intrinsic Matrix (K)
        # Calculates focal length based on FOV and image dimensions
        f_len = self.height / (2 * np.tan(np.radians(self.fov) / 2))
        self.K = np.array([
            [f_len, 0, self.width/2],
            [0, f_len, self.height/2],
            [0, 0, 1]
        ])
        self.K_inv = np.linalg.inv(self.K)
        
        # Projection matrix for PyBullet
        self.proj_mat = p.computeProjectionMatrixFOV(self.fov, width/height, self.near, self.far)

    def get_overhead_view(self):
        """Fixed overhead camera for global tabletop detection."""
        eye = [0.5, 0, 1.2]     # Camera Position
        target = [0.5, 0, 0]    # Looking at table center
        up = [0, 1, 0]          # Up vector
        return p.computeViewMatrix(eye, target, up)

    def get_wrist_view(self):
        """Active wrist-mounted camera for high-precision local detection."""
        # Using Panda Grasptarget (Link 11) for camera mounting
        state = p.getLinkState(self.robot_id, 11)
        pos, orn = state[4], state[5]
        rot = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        
        # Offset camera slightly from the flange
        eye = np.array(pos) + rot @ np.array([0, 0, 0.05])
        target = eye + rot @ np.array([0, 0, 1])
        up = rot @ np.array([-1, 0, 0])
        return p.computeViewMatrix(eye, target, up)

    def back_project(self, u, v, depth_val, view_matrix):
        """
        TECHNICAL REQUIREMENT: 3D Back-Projection
        Implements P_world = T_camera_world * K^-1 * P_pixel * d
        """
        # 1. Transform Screen to Normalized Device Coordinates (NDC)
        x = (2.0 * u / self.width) - 1.0
        y = 1.0 - (2.0 * v / self.height)
        z = (2.0 * depth_val) - 1.0 
        
        # 2. Correctly calculate Camera Extrinsic (Inverse View Matrix)
        view_mat = np.array(view_matrix).reshape(4, 4).T
        proj_mat = np.array(self.proj_mat).reshape(4, 4).T
        
        # T_camera_world is the inverse of the view-projection sequence
        inv_mvp = np.linalg.inv(proj_mat @ view_mat)
        
        # 3. P_world calculation
        p_homog = inv_mvp @ np.array([x, y, z, 1.0])
        return p_homog[:3] / p_homog[3]

    def capture(self, view_mat):
        """Captures RGB, Depth, and Segmentation masks."""
        _, _, rgb, d_buf, seg = p.getCameraImage(self.width, self.height, view_mat, self.proj_mat)
        return np.array(rgb), np.array(d_buf).reshape(self.height, self.width), \
               np.array(seg).reshape(self.height, self.width)
