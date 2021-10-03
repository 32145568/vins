cmake_minimum_required(VERSION 2.8.3)
project(back_end)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    message_generation
)

add_message_files(FILES kf.msg)  
add_message_files(FILES up_kf.msg)  
generate_messages(DEPENDENCIES sensor_msgs) 

find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(Ceres REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_executable(back_end 
    src/back_end_node.cpp
    src/parameter.cpp
    src/optimization.cpp
)

target_link_libraries(back_end ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


