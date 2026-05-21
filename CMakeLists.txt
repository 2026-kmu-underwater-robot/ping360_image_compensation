cmake_minimum_required(VERSION 3.8)
project(ping360_image_compensation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ping360_sonar_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(ping360_image_compensation_node
  src/main.cpp
  src/ping360_image_compensation.cpp
)

target_compile_features(ping360_image_compensation_node PUBLIC cxx_std_17)
target_include_directories(ping360_image_compensation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ping360_image_compensation_node
  image_transport
  nav_msgs
  ping360_sonar_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS ping360_image_compensation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
