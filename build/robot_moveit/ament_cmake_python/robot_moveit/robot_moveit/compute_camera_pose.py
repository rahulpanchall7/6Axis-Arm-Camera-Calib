#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from math import cos, sin, pi
from scipy.spatial.transform import Rotation as R

# Define UR5 DH parameters (standard values)
UR5_DH_PARAMS = [
    [0, 0, 0.089159, 0],          # Joint 1
    [-pi/2, -0.425, 0, 0],        # Joint 2
    [0, -0.39225, 0, 0],          # Joint 3
    [pi/2, 0, 0.10915, 0],        # Joint 4
    [-pi/2, 0, 0.09465, 0],       # Joint 5
    [0, 0, 0.0823, 0]             # Joint 6
]

class ComputeCameraPose(Node):
    def __init__(self):
        super().__init__('compute_camera_pose')
        
        # Transformation from end-effector to camera
        self.camera_to_ee_transform = np.array([
            [1, 0, 0, 0.05],
            [0, 1, 0, 0],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1]
        ])
        
        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # Initialize joint angles and last pose
        self.joint_angles = [0.0] * 6
        self.last_position = None
        self.last_orientation = None

    def dh_to_transform(self, alpha, a, d, theta):
        """Convert DH parameters to a transformation matrix."""
        ct, st = cos(theta), sin(theta)
        ca, sa = cos(alpha), sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles):
        """Compute the end-effector's pose using forward kinematics."""
        T = np.eye(4)
        for i, (alpha, a, d, theta_offset) in enumerate(UR5_DH_PARAMS):
            theta = joint_angles[i] + theta_offset
            T = np.dot(T, self.dh_to_transform(alpha, a, d, theta))
        return T

    def joint_states_callback(self, msg):
        """Callback function for joint states."""
        # Update joint angles
        self.joint_angles = msg.position[:6]
        
        # Compute end-effector's pose in the world frame
        ee_to_world = self.forward_kinematics(self.joint_angles)
        
        # Compute camera's pose in the world frame
        camera_to_world = np.dot(ee_to_world, self.camera_to_ee_transform)
        
        # Extract position and orientation
        position = camera_to_world[:3, 3]
        orientation = camera_to_world[:3, :3]  # Rotation matrix
        
        # Convert rotation matrix to Euler angles (roll, pitch, yaw)
        euler_angles = R.from_matrix(orientation).as_euler('xyz', degrees=True)
        
        # Ensure small negative values display as positive zero
        position = np.where(np.abs(position) < 1e-6, 0.0, position)
        euler_angles = np.where(np.abs(euler_angles) < 1e-6, 0.0, euler_angles)
        
        # Check for significant changes in position or orientation
        position_change = np.linalg.norm(position - self.last_position) if self.last_position is not None else float('inf')
        orientation_change = np.linalg.norm(euler_angles - self.last_orientation) if self.last_orientation is not None else float('inf')
        
        # Thresholds for significant changes
        position_threshold = 0.01  # 1 cm
        orientation_threshold = 1.0  # 1 degree
        
        # Log the pose if there is a significant change
        if position_change > position_threshold or orientation_change > orientation_threshold:
            # Format position and orientation for cleaner output
            position_str = f"[{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]"
            orientation_str = f"[{euler_angles[0]:.2f}, {euler_angles[1]:.2f}, {euler_angles[2]:.2f}]"
            
            self.get_logger().info(f"Camera Pose - Position (x, y, z): {position_str}")
            self.get_logger().info(f"Camera Pose - Orientation (Roll, Pitch, Yaw): {orientation_str}")
            
            # Update last pose
            self.last_position = position
            self.last_orientation = euler_angles

def main(args=None):
    rclpy.init(args=args)
    node = ComputeCameraPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
