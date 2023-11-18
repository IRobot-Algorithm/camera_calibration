cmake_minimum_required(VERSION 3.0.2)
project(image_publisher)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(cv_bridge_DIR "/home/niuoruo/bag/cv_bridge/devel/share/cv_bridge/cmake")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
)

# OpenCV
find_package(OpenCV 4.4.0 REQUIRED)
message(STATUS "Find OpenCV include at ${OpenCV_INCLUDE_DIRS}")
message(STATUS "Find OpenCV libraries: ${OpenCV_LIBRARIES}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ./general
  ./include
  ./mercure/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/mercure
)


# DahengCamera
add_executable(DahengCamera
  ./src/node/DahengCamera.cpp
  ./src/PubImage.cpp
)
target_link_libraries(DahengCamera 
  mercure_driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Video
add_executable(Video
  ./src/node/Video.cpp
  ./src/PubImage.cpp
)
target_link_libraries(Video
  mercure_driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

