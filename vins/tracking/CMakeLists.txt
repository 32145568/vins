cmake_minimum_required(VERSION 2.8.3)
project(tracking)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_executable(tracking 
    src/tracking_node.cpp
    src/tracking.cpp
    src/parameter.cpp
)

target_link_libraries(tracking ${catkin_LIBRARIES} ${OpenCV_LIBS})


