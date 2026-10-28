cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Cari semua dependensi yang diperlukan
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(vision_node src/vision.cpp)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()