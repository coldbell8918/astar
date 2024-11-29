cmake_minimum_required(VERSION 3.5)
project(astar_b9)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(astar_b9 src/astar_b9.cpp)

target_link_libraries(astar_b9 Eigen3::Eigen)

# Link libraries
ament_target_dependencies(astar_b9
  rclcpp
  std_msgs
  nav_msgs
  Eigen3
)

# Install target
install(TARGETS
  astar_b9
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
