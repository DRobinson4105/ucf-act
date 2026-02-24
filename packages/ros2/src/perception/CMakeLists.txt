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
  TARGETS seg_node lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
