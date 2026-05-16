cmake_minimum_required(VERSION 3.8)
project(geojson_reader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(askar_interfaces REQUIRED)

add_executable(geojson_reader_node src/geojson_reader_node.cpp)

ament_target_dependencies(geojson_reader_node
  rclcpp
  geometry_msgs
  askar_interfaces
)

install(TARGETS geojson_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
