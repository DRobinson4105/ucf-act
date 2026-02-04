cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(convert_odom_frame_lib src/convert_odom_frame.cpp)

target_include_directories(
  convert_odom_frame_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  convert_odom_frame_lib
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(
  convert_odom_frame_node
  src/convert_odom_frame.cpp
)

target_link_libraries(
  convert_odom_frame_node
  convert_odom_frame_lib
)

ament_target_dependencies(
  convert_odom_frame_node
  rclcpp
)

install(
  TARGETS convert_odom_frame_lib convert_odom_frame_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(convert_odom_frame_lib)
ament_export_dependencies(rclcpp nav_msgs tf2 tf2_ros tf2_geometry_msgs)

ament_package()
