cmake_minimum_required(VERSION 3.8)
project(image_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# 查找 Boost Asio 系统库
find_package(Boost REQUIRED COMPONENTS system)

# 编译为共享库 (Shared Library)
add_library(serial_transmitter_component SHARED
  src/serial_transmitter_node.cpp
)

# 链接目标依赖
ament_target_dependencies(serial_transmitter_component
  rclcpp
  std_msgs
  sensor_msgs
  rclcpp_components
)
target_include_directories(serial_transmitter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial_transmitter_component Boost::system)

rclcpp_components_register_node(serial_transmitter_component
  PLUGIN "image_serial::SerialTransmitterNode"
  EXECUTABLE serial_transmitter_node_exec
)


install(TARGETS
  serial_transmitter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
