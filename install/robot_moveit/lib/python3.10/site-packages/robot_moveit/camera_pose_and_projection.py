#!/usr/bin/env python3

import cv2
import numpy as np
import os
import rclpy
from rclpy.node import Node
import argparse  # Import argparse for command-line arguments

class CameraPoseAndProjection(Node):
    def __init__(self, image_folder):
        super().__init__('camera_pose_and_projection')
        self.get_logger().info(f"Initializing CameraPoseAndProjection with folder: {image_folder}")
        self.image_folder = image_folder
        self.camera_matrix = np.array([
            [381.00093,   0.     , 319.24565],
            [0.     , 381.03542, 239.83069],
            [0.     ,   0.     ,   1.     ]
        ])
        self.distortion_coeffs = np.array([-0.000222, -0.000229, 0.000068, -0.000014, 0.000000])
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.CHECKERBOARD = (8, 6)
        self.square_size = 0.02
        self.T_cam_world = None

    def load_images(self):
        self.get_logger().info(f"Loading images from folder: {self.image_folder}")
        
        # Check if the folder exists and is accessible
        if not os.path.isdir(self.image_folder):
            self.get_logger().error(f"Error: The folder {self.image_folder} does not exist or is inaccessible.")
            return []

        image_files = [os.path.join(self.image_folder, f) for f in os.listdir(self.image_folder) if f.endswith('.png')]
        
        if len(image_files) == 0:
            self.get_logger().warn("No PNG images found in the folder. Please check the path and file types.")
        # else:
        #     self.get_logger().info(f"Found images: {image_files}")
        
        return image_files

    def calibrate_camera(self):
        self.get_logger().info("Starting camera calibration...")
        objp = np.zeros((self.CHECKERBOARD[0] * self.CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.CHECKERBOARD[1], 0:self.CHECKERBOARD[0]].T.reshape(-1, 2) * self.square_size

        objpoints = []  # 3D points in real world space
        imgpoints = []  # 2D points in image plane
        image_files = self.load_images()

        for fname in image_files:
            img = cv2.imread(fname)  # Read image
            if img is None:
                self.get_logger().error(f"Failed to load image {fname}")
                continue
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert to grayscale
            ret, corners = cv2.findChessboardCorners(gray, self.CHECKERBOARD, None)  # Detect checkerboard corners

            if ret:
                objpoints.append(objp)
                refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(refined_corners)
                cv2.drawChessboardCorners(img, self.CHECKERBOARD, refined_corners, ret)
                cv2.imshow('Calibration', img)
                cv2.waitKey(500)  # Wait for 500ms to display the image
            else:
                self.get_logger().warn(f"Chessboard corners not found in image: {fname}")

        cv2.destroyAllWindows()  # Close the image window
        self.get_logger().info("Camera calibration complete.")

        # If we have enough points, calculate the camera pose
        if len(imgpoints) > 0:
            success, rvec, tvec = cv2.solvePnP(objp, imgpoints[0], self.camera_matrix, self.distortion_coeffs)
            if success:
                rotation_matrix, _ = cv2.Rodrigues(rvec)  # Convert rotation vector to rotation matrix
                self.T_cam_world = np.eye(4)
                self.T_cam_world[:3, :3] = rotation_matrix  # Set the rotation part
                self.T_cam_world[:3, 3] = tvec.T.flatten()  # Set the translation part
                self.get_logger().info("Camera pose successfully calibrated.")
            else:
                self.get_logger().error("Camera pose calibration failed.")
        else:
            self.get_logger().error("Not enough points to calibrate the camera.")

    def transform_world_to_camera(self, world_coords):
        self.get_logger().info(f"Transforming world coordinates {world_coords} to camera frame...")
        world_homogeneous = np.array([*world_coords, 1])  # Convert to homogeneous coordinates
        cam_coords = np.linalg.inv(self.T_cam_world) @ world_homogeneous  # Apply the inverse transformation
        return cam_coords[:3]  # Return only the 3D coordinates (ignore homogeneous component)

    def project_to_image(self, object_cam_coords):
        self.get_logger().info(f"Projecting camera coordinates {object_cam_coords} to image plane...")
        cam_homogeneous = np.array([*object_cam_coords, 1])  # Convert to homogeneous coordinates
        image_coords_homogeneous = self.camera_matrix @ cam_homogeneous[:3]  # Project to image plane
        u, v = image_coords_homogeneous[:2] / image_coords_homogeneous[2]  # Convert to pixel coordinates (u, v)
        return (u, v)

def main(args=None):
    print("Initializing CAmPoseProj node...")
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Camera Pose and Projection Node")
    parser.add_argument('image_folder', type=str, help="Path to the folder containing calibration images")
    parsed_args = parser.parse_args()

    image_folder = parsed_args.image_folder  # Get the image folder from the command-line argument

    # Initialize the CameraPoseAndProjection node with the provided folder path
    camera_pose = CameraPoseAndProjection(image_folder)
    camera_pose.calibrate_camera()

    # Test object transformations
    object_world = np.array([0.5, 0.0, 0.0])  # Example 3D world coordinates
    object_cam = camera_pose.transform_world_to_camera(object_world)  # Transform to camera frame
    u, v = camera_pose.project_to_image(object_cam)  # Project the 3D camera coordinates to 2D image coordinates

    # Print the results using ROS2 logging
    camera_pose.get_logger().info(f"Object Position in World Frame: {object_world}")
    camera_pose.get_logger().info(f"Object Position in Camera Frame: {object_cam}")
    camera_pose.get_logger().info(f"Projected 2D Image Coordinates: u = {u}, v = {v}")

    # Keep the node running
    rclpy.spin(camera_pose)

    # Shut down ROS2 when done
    rclpy.shutdown()

if __name__ == '__main__':
    main()
