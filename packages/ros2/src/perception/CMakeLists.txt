cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(
  trt_engine SHARED
  src/engine.cpp
  src/util.cpp
)

target_include_directories(
  trt_engine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${CUDAToolkit_INCLUDE_DIRS}
)

target_link_libraries(
  trt_engine PUBLIC
  ${OpenCV_LIBRARIES}
  CUDA::cudart
  nvinfer
  nvinfer_plugin
  nvonnxparser
)

add_executable(seg_node src/seg_node.cpp)

target_link_libraries(seg_node trt_engine)

target_include_directories(seg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(seg_node
  rclcpp
  cv_bridge
  sensor_msgs
  OpenCV
)

add_executable(seg_cloud_node src/seg_cloud_node.cpp)

target_include_directories(
  seg_cloud_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(seg_cloud_node
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  tf2_eigen
)

add_executable(det_node src/det_node.cpp)

target_link_libraries(det_node trt_engine)

target_include_directories(det_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(det_node
  rclcpp
  cv_bridge
  sensor_msgs
  OpenCV
  vision_msgs
)

add_executable(lidar_filter_node src/lidar_filter_node.cpp)

target_link_libraries(lidar_filter_node ${PCL_LIBRARIES})

target_include_directories(lidar_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(lidar_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(
  TARGETS seg_node seg_cloud_node det_node lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
