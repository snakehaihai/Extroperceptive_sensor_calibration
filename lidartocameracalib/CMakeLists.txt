cmake_minimum_required(VERSION 2.8.3)
project(lidartocameracalib)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
  roslib
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
    cv_bridge
    camera_model
    visualization_msgs
    )

SET("MRPT_DIR" "/home/snake/library/mrpt-mrpt-1.5/build/")
find_package(OpenCV REQUIRED)
find_package(MRPT 1.5 REQUIRED gui obs slam)
find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

add_executable(lidartocameracalib
    src/lidartocameracalib.cpp

    )

#target_include_directories(lidartocameracalib PUBLIC "/home/snake/library/mrpt-mrpt-1.5/libs" /home/snake/library/mrpt-mrpt-1.5/build/lib)
SET("MRPT_INCLUDE_DIRS" "/home/snake/library/mrpt-mrpt-1.5/build/libs")
include_directories(${CERES_INCLUDE_DIRS} ${MRPT_INCLUDE_DIRS})




target_link_libraries(lidartocameracalib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${MRPT_LIBS} )


