cmake_minimum_required(VERSION 3.8)
project(mie443_contest2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required dependencies (available on both laptop and Pi)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Find optional dependencies (may not be available on Pi)
find_package(moveit_ros_planning_interface QUIET)
find_package(moveit_msgs QUIET)
find_package(nav2_msgs QUIET)
find_package(nav2_util QUIET)
find_package(orocos_kdl QUIET)

# Check if MoveIt is available
if(moveit_ros_planning_interface_FOUND AND moveit_msgs_FOUND)
  set(MOVEIT_AVAILABLE TRUE)
  message(STATUS "MoveIt found - building full contest2 executable")
else()
  set(MOVEIT_AVAILABLE FALSE)
  message(STATUS "MoveIt NOT found - building lightweight components only (image_capture_server)")
endif()

# Check if Nav2 is available
if(nav2_msgs_FOUND AND nav2_util_FOUND)
  set(NAV2_AVAILABLE TRUE)
  message(STATUS "Nav2 found")
else()
  set(NAV2_AVAILABLE FALSE)
  message(STATUS "Nav2 NOT found")
endif()

# Generate custom messages and services (always needed)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloDetection.msg"
  "srv/CaptureImage.srv"
  "srv/DetectObject.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# ============================================================================
# LIGHTWEIGHT COMPONENTS (build on both laptop and Pi)
# ============================================================================

# Base dependencies for lightweight components
set(base_dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  ament_index_cpp
)

# Image capture service server (runs on robot/Pi)
add_executable(image_capture_server
  src/image_capture_server.cpp
)

add_dependencies(image_capture_server ${PROJECT_NAME}__rosidl_generator_cpp)
ament_target_dependencies(image_capture_server ${base_dependencies})
rosidl_get_typesupport_target(cpp_typesupport_target_server ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(image_capture_server "${cpp_typesupport_target_server}" ${OpenCV_LIBRARIES})

# Install lightweight executables (always)
install(TARGETS
  image_capture_server
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# FULL COMPONENTS (build only on laptop with MoveIt/Nav2)
# ============================================================================

if(MOVEIT_AVAILABLE AND NAV2_AVAILABLE)
  include_directories(${orocos_kdl_INCLUDE_DIRS})

  # Full dependencies including MoveIt and Nav2
  set(full_dependencies
    ${base_dependencies}
    moveit_ros_planning_interface
    moveit_msgs
    nav2_msgs
    nav2_util
  )

  # Contest2 executable (requires MoveIt)
  add_executable(contest2
    src/contest2.cpp
    src/boxes.cpp
    src/navigation.cpp
    src/robot_pose.cpp
    src/yoloInterface.cpp
    src/arm_controller.cpp
    src/apriltag_detector.cpp
  )

  # Ensure messages are generated before compilation
  add_dependencies(contest2 ${PROJECT_NAME}__rosidl_generator_cpp)

  ament_target_dependencies(contest2 ${full_dependencies})

  target_link_libraries(contest2
    ${OpenCV_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
  )

  # Link custom messages to contest2
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(contest2 "${cpp_typesupport_target}")

  # Install contest2 executable
  install(TARGETS
    contest2
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ============================================================================
# COMMON INSTALLS (always)
# ============================================================================

# Install Python scripts
install(PROGRAMS
  src/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  boxes_database
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
