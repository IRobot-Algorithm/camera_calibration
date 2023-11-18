cmake_minimum_required(VERSION 3.8)
project(rmos_cam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
include_directories(include)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_add_library(TEST SHARED
  DIRECTORY test
)

target_link_libraries(${PROJECT_NAME} gxiapi pthread ${OpenCV_LIBS})
target_link_libraries(TEST gxiapi pthread ${OpenCV_LIBS})

# 注册component节点
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rmos_cam::DahengCamNode
  EXECUTABLE daheng_camera
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rmos_cam::USBCamNode
  EXECUTABLE usb_camera
)

rclcpp_components_register_node(TEST
  PLUGIN rmos_cam::CamSubNode
  EXECUTABLE cam_sub
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)