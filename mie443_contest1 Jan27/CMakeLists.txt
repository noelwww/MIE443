cmake_minimum_required(VERSION 3.8)
project(mie443_contest1)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Contest1 executable
add_executable(contest1 src/contest1.cpp)

ament_target_dependencies(contest1
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  irobot_create_msgs
  tf2
  tf2_geometry_msgs
  cv_bridge
  image_transport
)

target_link_libraries(contest1
  ${OpenCV_LIBRARIES}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Install executable
install(TARGETS
  contest1
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
