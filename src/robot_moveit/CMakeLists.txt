cmake_minimum_required(VERSION 3.22)
project(robot_moveit)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python scripts with executable permissions
install(PROGRAMS
  ${PROJECT_NAME}/ur5_controller.py
  ${PROJECT_NAME}/compute_camera_pose.py
  ${PROJECT_NAME}/camera_pose_and_projection.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)



install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()


