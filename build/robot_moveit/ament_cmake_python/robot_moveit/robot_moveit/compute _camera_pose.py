#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from math import cos, sin, pi

# Define UR5 DH parameters (standard values)
# Format: [alpha, a, d, theta_offset]
# Source: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
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

        # Define the transformation from the end-effector to the camera
        # Example: Camera is offset by 0.05m in x and 0.1m in z relative to the end-effector
        self.camera_to_ee_transform = np.array([
            [1, 0, 0, 0.05],
            [0, 1, 0, 0],
            [0, 0, 1, 0.1],
            [0, 0, 0, 1]
        ])

        # Subscribe to the joint states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # Topic name for joint states
            self.joint_states_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Initialize joint angles
        self.joint_angles = [0.0] * 6

    def dh_to_transform(self, alpha, a, d, theta):
        """Convert DH parameters to a transformation matrix."""
        ct = cos(theta)
        st = sin(theta)
        ca = cos(alpha)
        sa = sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        """Compute the end-effector's pose using forward kinematics."""
        T = np.eye(4)  # Start with the identity matrix
        for i, (alpha, a, d, theta_offset) in enumerate(UR5_DH_PARAMS):
            theta = joint_angles[i] + theta_offset
            Ti = self.dh_to_transform(alpha, a, d, theta)
            T = np.dot(T, Ti)
        return T

    def joint_states_callback(self, msg):
        """Callback function for joint states."""
        # Extract joint angles from the message
        # Ensure the order matches the UR5 joint order: ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3']
        self.joint_angles = msg.position[:6]

        # Compute the end-effector's pose in the world frame
        ee_to_world = self.forward_kinematics(self.joint_angles)

        # Compute the camera's pose in the world frame
        camera_to_world = np.dot(ee_to_world, self.camera_to_ee_transform)

        # Extract position and orientation
        position = camera_to_world[:3, 3]
        orientation = camera_to_world[:3, :3]  # Rotation matrix

        # Log the camera's pose
        self.get_logger().info(f"Camera Position in World Frame: {position}")
        self.get_logger().info(f"Camera Orientation in World Frame:\n{orientation}")

def main(args=None):
    rclpy.init(args=args)
    node = ComputeCameraPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()