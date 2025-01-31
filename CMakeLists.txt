cmake_minimum_required(VERSION 3.8)
project(rviz_gazebo_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

####package as library
# Include directories
include_directories(
  include
)

# Create a shared library
add_library(${PROJECT_NAME} SHARED
  src/rviz_gazebo_bridge.cpp
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ignition-msgs8
  ignition-transport11
  tf2_geometry_msgs
  geometric_shapes
  moveit_ros_planning_interface
)

# Export include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
####

install(DIRECTORY models DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
