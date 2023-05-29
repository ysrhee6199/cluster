cmake_minimum_required(VERSION 3.5)
project(cluster)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable CUDA
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("embed_platform on")
  include_directories(/usr/local/cuda/targets/aarch64-linux/include)
  link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
else()
  message("embed_platform off")
  # cuda
  include_directories(/usr/local/cuda/include)
  link_directories(/usr/local/cuda/lib64)

  # tensorrt
  include_directories(/home/nvidia/TensorRT-8.2.5.1/include)
  link_directories(/home/nvidia/TensorRT-8.2.5.1/lib)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(rtx_msg_interface REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(ament_cmake_auto REQUIRED)




include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)
include_directories(include/dummy_inference/include/yolov7/include)
include_directories(include/dummy_inference/include/yolov7/plugin)

ament_auto_find_build_dependencies()






# Build
# Nodes
ament_auto_add_library(cluster SHARED
  src/cluster.cpp
  include/cluster_inference/cluster_inference.cpp
  include/dummy_inference/yolov7.cpp
  include/pcan/ObjectDetectionsSender.cpp
  include/dummy_inference/include/yolov7/plugin/yololayer.cu
  include/dummy_inference/include/yolov7/src/block.cpp
  include/dummy_inference/include/yolov7/src/calibrator.cpp
  include/dummy_inference/include/yolov7/src/model.cpp
  include/dummy_inference/include/yolov7/src/postprocess.cpp
  include/dummy_inference/include/yolov7/src/preprocess.cu
)

add_library(tensorrt_plugins SHARED
  include/dummy_inference/include/yolov7/plugin/yololayer.cu
  include/dummy_inference/include/yolov7/src/block.cpp
  include/dummy_inference/include/yolov7/src/calibrator.cpp
  include/dummy_inference/include/yolov7/src/model.cpp
  include/dummy_inference/include/yolov7/src/postprocess.cpp
  include/dummy_inference/include/yolov7/src/preprocess.cu
)

target_link_libraries(cluster
  nvinfer
  cudart
  tensorrt_plugins
  pcanbasic
)



ament_auto_add_executable(cluster_exec
  src/exec/cluster_exec.cpp
)
#install


ament_auto_package(INSTALL_TO_SHARE
  calibrations
  config
  launch
)
