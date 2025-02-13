**Overview:**

This project aims to develop a Python program to control a UR5 6-axis robotic arm equipped with a camera at the end effector, using forward kinematics to compute the arm’s position and orientation in the world frame. The program will determine the camera’s 3D pose relative to the world frame through kinematic modeling and matrix transformations while enabling precise control of the robotic arm for positioning and movement.

**System Requirements:**

Ubuntu 22.04
Ros2 Iron Iriwni

**Dependencies:**

sudo apt-get install -y ros-iron-joint-state-publisher-gui ros-iron-xacro ros-iron-ros-gz* ros-iron-ros2-control ros-iron-ros2-controllers ros-iron-moveit* python3-pip

**Steps:**

1. ros2 launch robot_description gazebo.launch.py
2. ros2 launch robot_description display.launch.py
3. ros2 launch robot_controller controller.launch.py
4. ros2 run robot_moveit ur5_controller.py <shoulder_pan_joint> <shoulder_lift_joint> <elbow_joint> <wrist_1_joint> <wrist_2_joint> <wrist_3_joint>
5. ros2 run cobot_moveit ur5_controller.py --reset_home (to bring back to home position)
6. ros2 run robot_description compute_camera_pose.py
