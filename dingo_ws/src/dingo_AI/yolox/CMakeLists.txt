cmake_minimum_required(VERSION 3.0.2)
project(yolox)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nodelet
  pluginlib
)
find_package(OpenCV REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt
)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS /host_cuda/include /usr/local/cuda/include
)
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/aarch64-linux-gnu)
find_library(CUDART_LIB cudart PATHS /host_cuda/lib64 /usr/local/cuda/lib64 /usr/lib/aarch64-linux-gnu)

if(NOT TENSORRT_INCLUDE_DIR OR NOT CUDA_INCLUDE_DIR OR NOT NVINFER_LIB OR NOT CUDART_LIB)
  message(FATAL_ERROR "TensorRT/CUDA runtime libraries not found")
endif()

catkin_package(
  LIBRARIES yolo_trt_nodelet
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
)

add_executable(yolo_trt_node src/yolo_trt_node.cpp)
add_library(yolo_trt_nodelet src/yolo_trt_nodelet.cpp)

target_link_libraries(yolo_trt_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${NVINFER_LIB}
  ${CUDART_LIB}
)
target_link_libraries(yolo_trt_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${NVINFER_LIB}
  ${CUDART_LIB}
)
set_target_properties(yolo_trt_node PROPERTIES
  BUILD_RPATH "/host_cuda/lib64;/host_cuda/targets/aarch64-linux/lib"
  INSTALL_RPATH "/host_cuda/lib64;/host_cuda/targets/aarch64-linux/lib"
)
set_target_properties(yolo_trt_nodelet PROPERTIES
  BUILD_RPATH "/host_cuda/lib64;/host_cuda/targets/aarch64-linux/lib"
  INSTALL_RPATH "/host_cuda/lib64;/host_cuda/targets/aarch64-linux/lib"
)

catkin_install_python(PROGRAMS
  scripts/yolo_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS yolo_trt_node yolo_trt_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/run_yolo_tracker.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
