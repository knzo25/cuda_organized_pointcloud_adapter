cmake_minimum_required(VERSION 3.14)
project(cuda_organized_pointcloud_adapter)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)
find_package(PCL REQUIRED)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found, so the cuda_organized_pointcloud_adapter package will not be built.")
  return()
endif()

ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

# Targets
ament_auto_add_library(cuda_organized_pointcloud_adapter SHARED
  src/cuda_organized_pointcloud_adapter/cuda_organized_pointcloud_adapter_node.cpp
)

target_link_libraries(cuda_organized_pointcloud_adapter
  ${CUDA_LIBRARIES}
)

rclcpp_components_register_node(cuda_organized_pointcloud_adapter
  PLUGIN "cuda_organized_pointcloud_adapter::CudaOrganizedPointcloudAdapterNode"
  EXECUTABLE cuda_organized_pointcloud_adapter_node
)

ament_auto_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
