cmake_minimum_required(VERSION 3.8)
project(replanner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(askar_interfaces REQUIRED)

include_directories(include)

add_executable(replanner_node
  src/replanner_node.cpp
  src/dstar_lite.cpp
)

ament_target_dependencies(replanner_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  askar_interfaces
)

install(TARGETS replanner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
