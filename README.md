**Overview:**

This project aims to develop a Python program to control a UR5 6-axis robotic arm equipped with a camera at the end effector, using forward kinematics to compute the arm’s position and orientation in the world frame. The program will determine the camera’s 3D pose relative to the world frame through kinematic modeling and matrix transformations while enabling precise control of the robotic arm for positioning and movement.






**System Requirements:**

Ubuntu 22.04
Ros2 Iron Iriwni

**Dependencies:**

sudo apt-get install -y ros-iron-joint-state-publisher-gui ros-iron-xacro ros-iron-ros-gz* ros-iron-ros2-control ros-iron-ros2-controllers ros-iron-moveit* python3-pip

**Steps:**

1. ros2 launch robot_description gazebo.launch.py
![image](https://github.com/user-attachments/assets/9701cf08-2b6b-4d31-a6e3-07f2c43435a8)

2. ros2 launch robot_description display.launch.py
![image](https://github.com/user-attachments/assets/7a2748ae-d5e7-47c9-94a1-be89eec8f50a)

3. ros2 launch robot_controller controller.launch.py
4. ros2 run robot_moveit ur5_controller.py <shoulder_pan_joint> <shoulder_lift_joint> <elbow_joint> <wrist_1_joint> <wrist_2_joint> <wrist_3_joint>
![image](https://github.com/user-attachments/assets/9ce7755c-8776-418f-b4e9-2abbaf361a8b)

5. ros2 run cobot_moveit ur5_controller.py --reset_home (to bring back to home position)  
6. ros2 run robot_description compute_camera_pose.py
![image](https://github.com/user-attachments/assets/285e3fdb-0393-47e8-9c57-241db344d561)



