cmake_minimum_required(VERSION 3.8)
project(global_planner)

# C++14 kullan
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Uyarıları aç
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(askar_interfaces REQUIRED)

# Include klasörünü tanıt
include_directories(include)

# Çalıştırılabilir dosyayı oluştur
add_executable(global_planner_node
  src/global_planner_node.cpp
  src/astar.cpp
)

# Bağımlılıkları bağla
ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  askar_interfaces
)

# Kurulum hedefi
install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Include klasörünü kur
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
