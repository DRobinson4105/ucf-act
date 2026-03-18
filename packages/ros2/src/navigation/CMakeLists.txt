cmake_minimum_required(VERSION 3.8)
project(navigation)

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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(cmd_vel_adapter src/cmd_vel_adapter.cpp)
ament_target_dependencies(cmd_vel_adapter
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_msgs
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

add_executable(clicked_point_path_publisher src/clicked_point_path_publisher.cpp)
ament_target_dependencies(clicked_point_path_publisher
  rclcpp
  geometry_msgs
  nav_msgs
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

add_executable(route_navigator src/route_navigator.cpp)
ament_target_dependencies(route_navigator
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)



install(TARGETS
  act_global_path_manager
  fake_odom_integrator
  cmd_vel_adapter
  mock_livox_pointcloud_publisher
  clicked_point_path_publisher
  follow_path_client
  route_navigator
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveCommand.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

rosidl_target_interfaces(cmd_vel_adapter
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
ament_export_dependencies(rosidl_default_runtime)


ament_package()
