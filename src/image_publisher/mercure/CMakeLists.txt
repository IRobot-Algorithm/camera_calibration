Cmake_minimum_required (VERSION 2.8)

project(mercure_driver)

message(STATUS "Configuring Sub_project:mercure_drive")

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
# )

find_package(OpenCV 4 REQUIRED)

include_directories(
   ./include
)
add_library(mercure_driver ./src/mercure_driver.cpp)

target_link_libraries(mercure_driver ${OpenCV_LIBS} gxiapi)


