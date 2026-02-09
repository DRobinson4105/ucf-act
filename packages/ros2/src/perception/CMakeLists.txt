cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/Array.msg"
  "msg/Array2D.msg"
  "msg/Array3D.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
  rclcpp::rclcpp
)

add_executable(segmentation_node src/segmentation_node.cpp)

target_link_libraries(
  segmentation_node PRIVATE
  trt_engine
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  cv_bridge::cv_bridge
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  segmentation_node PRIVATE
  ${cpp_typesupport_target}
)

install(
  TARGETS segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
