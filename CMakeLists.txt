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

# Find OHM Library built in local workspace
find_package(ohm REQUIRED)

# Build C++ OHM Occupancy Executable
add_executable(ohm_occupancy_node src/ohm_occupancy_node.cpp)
ament_target_dependencies(ohm_occupancy_node rclcpp sensor_msgs visualization_msgs)
target_link_libraries(ohm_occupancy_node ohm::ohm)

# Install C++ Executables
install(TARGETS ohm_occupancy_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python Package & Executable Scripts
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/standalone_voxel_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()