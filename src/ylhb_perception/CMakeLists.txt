cmake_minimum_required(VERSION 3.8)
project(ylhb_perception)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include
)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  HINTS /usr/local/cuda/include /usr/include
)
find_library(NVINFER_LIBRARY nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64
)
find_library(CUDART_LIBRARY cudart
  HINTS /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT CUDA_INCLUDE_DIR OR NOT NVINFER_LIBRARY OR NOT CUDART_LIBRARY)
  message(FATAL_ERROR "TensorRT/CUDA runtime libraries are required for yolo_detector_node")
endif()

add_executable(yolo_detector_node src/yolo_detector_node.cpp)
target_include_directories(yolo_detector_node PRIVATE
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(yolo_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(yolo_detector_node
  ${OpenCV_LIBRARIES}
  ${NVINFER_LIBRARY}
  ${CUDART_LIBRARY}
)

install(PROGRAMS
  scripts/yolo_detector_node.py
  scripts/object_localizer_node.py
  scripts/export_yolo_trt.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # TensorRT runtime code predates the current uncrustify profile; reformat it
  # separately from behavior changes.
  set(ament_cmake_uncrustify_FOUND TRUE)
  # xmllint fetches the ROS package schema over the network in this environment.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
