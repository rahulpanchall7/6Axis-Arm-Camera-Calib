cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED) 
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

install(
  DIRECTORY launch meshes urdf rviz 
  DESTINATION share/${PROJECT_NAME}
)

# # Export dependencies
# ament_export_dependencies(
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   tf2_ros
#   urdf
#   gazebo_ros
#   ros_gz_sim
#   ros_gz_bridge
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
