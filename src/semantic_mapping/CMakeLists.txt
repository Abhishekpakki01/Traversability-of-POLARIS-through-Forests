cmake_minimum_required(VERSION 3.8)
project(semantic_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 & C++ Build Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Find OHM Library built in local workspace
find_package(ohm REQUIRED)

# Build C++ OHM Occupancy Executable
add_executable(ohm_occupancy_node src/ohm_occupancy_node.cpp)
ament_target_dependencies(ohm_occupancy_node rclcpp sensor_msgs visualization_msgs)
target_link_libraries(ohm_occupancy_node ohm::ohm)

# Pose-aware variant; keep the original executable available for comparison.
add_executable(ohm_occupancy_pose_node src/ohm_occupancy_pose_node.cpp)
ament_target_dependencies(ohm_occupancy_pose_node rclcpp sensor_msgs visualization_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(ohm_occupancy_pose_node ohm::ohm)

# Install C++ Executables
install(TARGETS ohm_occupancy_node ohm_occupancy_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Package & Executable Scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/standalone_voxel_node.py
  scripts/camera_color_preview.py
  scripts/camera_lidar_projection_node.py
  scripts/camera_semantic_voxel_node.py
  scripts/tartan_camera_adapter.py
  scripts/tartan_recorded_camera_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(ohm_pose_mapping test/test_ohm_pose_mapping.py
    TIMEOUT 60
    ENV OHM_NODE_EXECUTABLE=$<TARGET_FILE:ohm_occupancy_pose_node>
  )
  ament_add_pytest_test(camera_projection test/test_camera_projection.py TIMEOUT 30)
  ament_add_pytest_test(semantic_fusion test/test_semantic_fusion.py TIMEOUT 30)
  ament_add_pytest_test(tartan_camera test/test_tartan_camera.py TIMEOUT 30)
  ament_add_pytest_test(recorded_calibration test/test_recorded_calibration.py TIMEOUT 30)
endif()

ament_package()
