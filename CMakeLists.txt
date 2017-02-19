cmake_minimum_required(VERSION 2.8.3)
project(teleop_vision)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)

option(WITH_ROS "Enable support for ROS topics in opencv_cam_calib" OFF)
if(WITH_ROS)
    add_definitions(-DWITH_ROS)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf_conversions
        cv_bridge
        image_transport
        )

find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES teleop_vision
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
        #CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        include/
)


# Bundle all the common functionality into a single library
add_library(
        arucoUtils
        src/utils/BoardDetector.cpp
)
target_link_libraries(
        arucoUtils
        ${OpenCV_LIBRARIES})

# List all the executables that we should build and link
#set(_executables teleop_ac_vision_node intrinsic_calibration )
#set(_executables intrinsic_calibration )
#foreach (_executable ${_executables})
#    add_executable(${_executable} src/${_executable}.cpp)
#    target_link_libraries(${_executable}
#            ${OpenCV_LIBRARIES}
#            ${catkin_LIBRARIES}
#            arucoUtils)
#endforeach ()

add_executable(
        intrinsic_calib
        src/intrinsic_calib/intrinsic_calibration.cpp
)

add_executable(
        robot_to_camera_aruco
        src/robot_to_camera_aruco/main_robot_to_camera_aruco.cpp
        src/robot_to_camera_aruco/RobotToCameraAruco.cpp
        src/utils/Drawings.cpp src/utils/Conversions.cpp)


add_executable(
        extrinsic_calib_aruco
        src/extrinsic_calib_aruco/main_extrinsic_aruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.cpp
        src/utils/Conversions.cpp)

add_executable(
        camera_capture_flea3
        src/camera_capture_flea3/main_camera_capture_flea3.cpp
        src/camera_capture_flea3/Fla3Camera.cpp
        src/camera_capture_flea3/Fla3Camera.h
)
# Find the Flycapture include files and libraries
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/src/flycapture)


# MESSAGE(${FLYCAPTURE2_INCLUDE_DIR})
find_package (Flycapture2 REQUIRED)
if (FLYCAPTURE2_FOUND)
    MESSAGE("FLYCAPTURE2 found!")
    include_directories(${FLYCAPTURE2_INCLUDE_DIR})
endif (FLYCAPTURE2_FOUND)

target_link_libraries(
        camera_capture_flea3
        ${FLYCAPTURE2_LIBRARIES})


add_executable(
        create_aruco_board
        src/utils/create_aruco_board.cpp
)
target_link_libraries(
        create_aruco_board
        ${OpenCV_LIBRARIES})


set(_executables intrinsic_calib extrinsic_calib_aruco robot_to_camera_aruco camera_capture_flea3)
foreach (_executable ${_executables})
    target_link_libraries(
            ${_executable}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)
endforeach ()


