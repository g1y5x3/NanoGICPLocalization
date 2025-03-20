cmake_minimum_required(VERSION 3.8)
project(pcd_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pcd_localization src/localization.cpp)
ament_target_dependencies(pcd_localization
  rclcpp
  sensor_msgs
)

install(TARGETS
  pcd_localization
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
