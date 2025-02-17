#!/usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class UR5Controller(Node):

    def __init__(self):
        super().__init__('ur5_controller')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.actual_positions = None  # Store last received joint positions

        # Define UR5 Joint Limits
        self.joint_limits = {
            "shoulder_pan_joint": (-3.14, 3.14),
            "shoulder_lift_joint": (-3.14, 3.14),
            "elbow_joint": (-3.14, 3.14),
            "wrist_1_joint": (-3.14, 3.14),
            "wrist_2_joint": (-3.14, 3.14),
            "wrist_3_joint": (-3.14, 3.14),
        }

    def check_joint_limits(self, angles):
        """ Validate if the given joint angles are within limits. """
        for i, angle in enumerate(angles):
            min_limit, max_limit = self.joint_limits[self.joint_names[i]]
            if not (min_limit <= angle <= max_limit):
                self.get_logger().error(f'Joint {self.joint_names[i]} out of limits! Given: {angle}, Allowed: ({min_limit}, {max_limit})')
                return False
        return True

    def send_goal(self, angles):
        """ Send goal only if the angles are within limits and the target is different. """
        if not self.check_joint_limits(angles):
            self.get_logger().error('Aborting: One or more joint angles are out of limits!')
            return False

        # Check if the goal angles are the same as the current position
        if self.actual_positions is not None and all(abs(a - b) < 1e-3 for a, b in zip(self.actual_positions, angles)):
            self.get_logger().info('Target position is the same as the current position. No movement needed.')
            return True  # Skip sending goal if the positions are the same

        # Calculate the trajectory if positions are different
        goal_msg = FollowJointTrajectory.Goal()
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=2, nanoseconds=0).to_msg()
        point.positions = angles
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]
        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available! Aborting...')
            return False

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the action server!')
            return

        self.get_logger().info('Goal accepted! Executing...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result

        if self.actual_positions is None:
            self.get_logger().error('No feedback received! Unable to verify goal execution.')
        else:
            tolerance = 0.2  # Increase tolerance if needed
            difference = [abs(actual - desired) for actual, desired in zip(self.actual_positions, self.goal_positions)]
            
            # Log actual vs. desired values for debugging
            self.get_logger().info(f'Desired Positions: {self.goal_positions}')
            self.get_logger().info(f'Actual Positions:  {self.actual_positions}')
            self.get_logger().info(f'Error per joint:   {difference}')

            if all(diff <= tolerance for diff in difference):
                self.get_logger().info('Goal successfully reached!')
            else:
                self.get_logger().error('Goal NOT reached! Consider increasing tolerance.')

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """ Store latest feedback joint positions. """
        feedback = feedback_msg.feedback
        self.actual_positions = feedback.actual.positions
        self.get_logger().info(f'Feedback received: {self.actual_positions}')

    def reset_to_home(self):
        """ Reset the robot to its home position. """
        home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Define UR5 home position
        self.get_logger().info('Resetting to home position...')
        self.send_goal(home_position)

def prompt_for_angles():
    """ Continuously prompt the user for valid input. """
    while True:
        try:
            user_input = input("Please enter 6 joint angles in radians (space-separated): ")
            angles = [float(angle) for angle in user_input.split()]
            print(f"Parsed angles: {angles}")  # Debugging line to see the parsed angles
            
            if len(angles) != 6:
                print("Error: Please provide exactly 6 joint angles.")
                continue

            # Validate joint limits
            if not all(isinstance(angle, float) for angle in angles):
                print("Error: All inputs must be valid floating point numbers.")
                continue

            return angles
        except ValueError:
            print("Error: Invalid input! Please enter numeric values.")
        except Exception as e:
            print(f"Unexpected error: {e}. Please try again.")

def main(args=None):
    # print("Initializing UR5Controller node...")
    rclpy.init()

    action_client = UR5Controller()

    # Check if '--reset_home' is provided
    if '--reset_home' in sys.argv:
        action_client.reset_to_home()
        return  # Exit after resetting to home

    # Check if command line arguments are provided
    if len(sys.argv) != 7:
        print("Error: Please provide exactly 6 joint angles in radians.")
        angles = prompt_for_angles()  # Ask the user to input angles if not provided correctly
    else:
        try:
            angles = [float(sys.argv[i]) for i in range(1, 7)]
            print(f"Parsed angles from command-line: {angles}")
        except ValueError:
            print("Error: Invalid input! Please enter numeric values.")
            rclpy.shutdown()
            return

    # Store goal positions
    action_client.goal_positions = angles

    # Send the goal to the robot
    success = action_client.send_goal(angles)

    if success:
        rclpy.spin(action_client)

if __name__ == '__main__':
    main()