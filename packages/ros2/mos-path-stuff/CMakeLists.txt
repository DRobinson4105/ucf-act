cmake_minimum_required(VERSION 3.8)
project(act_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(GeographicLib QUIET)

if(NOT GeographicLib_FOUND)
  list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
  find_package(GeographicLib REQUIRED)
endif()

if(TARGET GeographicLib::GeographicLib)
  set(_GEOLIB_TARGET GeographicLib::GeographicLib)
endif()

add_executable(act_global_path_manager src/act_global_path_manager.cpp)
ament_target_dependencies(act_global_path_manager
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

if(TARGET GeographicLib::GeographicLib)
  target_link_libraries(act_global_path_manager GeographicLib::GeographicLib)
else()
  target_include_directories(act_global_path_manager PRIVATE ${GeographicLib_INCLUDE_DIRS})
  target_link_libraries(act_global_path_manager ${GeographicLib_LIBRARIES})
endif()

add_executable(fake_odom_integrator src/fake_odom_integrator.cpp)
ament_target_dependencies(fake_odom_integrator
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(mock_livox_pointcloud_publisher src/mock_livox_pointcloud_publisher.cpp)
ament_target_dependencies(mock_livox_pointcloud_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(follow_path_client src/follow_path_client.cpp)
ament_target_dependencies(follow_path_client
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
)

install(TARGETS
  act_global_path_manager
  fake_odom_integrator
  mock_livox_pointcloud_publisher
  follow_path_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

