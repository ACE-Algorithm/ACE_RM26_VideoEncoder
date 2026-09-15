cmake_minimum_required(VERSION 3.8)
project(landing_point_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(cv_bridge REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/landing_point_detector_node.cpp
  src/codec_common.cpp
  src/packet_encoder.cpp
  src/packet_decoder.cpp
  src/temporal_frame_composer.cpp
  src/transport_packetizer.cpp
  src/receiver_presentation.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
# 如果需要链接其他库（如 OpenCV），ament_auto 通常会自动处理，
# 但有时需要显式指定链接目录或添加 include 目录。由于 package.xml 中已声明依赖，
# ament_auto 会自动添加它们的 include 路径和链接库，但为了保险可以手动添加：
target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  PkgConfig::FFMPEG
)

add_executable(ros2_packet_decode_receiver
  src/ros2_packet_decode_receiver.cpp
)
ament_target_dependencies(ros2_packet_decode_receiver
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(ros2_packet_decode_receiver
  ${PROJECT_NAME}
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "landing_point_detector::LandingPointDetectorNode"
  EXECUTABLE landing_point_detector_node
)

# 安装头文件（如果需要在其他包中使用）
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(TARGETS ros2_packet_decode_receiver
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
