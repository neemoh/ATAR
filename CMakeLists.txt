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
        src/utils/Conversions.cpp
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
        src/utils/Drawings.cpp )


add_executable(
        extrinsic_calib_aruco
        src/extrinsic_calib_aruco/main_extrinsic_aruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.cpp
        )


# Find the Flycapture include files and libraries
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/src/flycapture)


# MESSAGE(${FLYCAPTURE2_INCLUDE_DIR})
find_package (Flycapture2)
if (FLYCAPTURE2_FOUND)
    	MESSAGE("FLYCAPTURE2 found.")
    	include_directories(${FLYCAPTURE2_INCLUDE_DIR})

	add_executable(
        	camera_capture_flea3
        	src/camera_capture_flea3/main_camera_capture_flea3.cpp
        	src/camera_capture_flea3/Fla3Camera.cpp
        	src/camera_capture_flea3/Fla3Camera.h
		)
	target_link_libraries(
	        camera_capture_flea3
        	${FLYCAPTURE2_LIBRARIES}
		)
    target_link_libraries(
            camera_capture_flea3
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)

endif (FLYCAPTURE2_FOUND)



add_executable(
        create_aruco_board
        src/utils/create_aruco_board.cpp
)
target_link_libraries(
        create_aruco_board
        ${OpenCV_LIBRARIES})

find_package(OpenGL)
find_package(glfw3)
if(GLFW3_FOUND)
    MESSAGE("GLFW3 found.")
    include_directories(${GLFW3_INCLUDE_DIR})
	add_executable(
		    overlay
		src/overlay/main_overlay.cpp
		src/overlay/OverlayGraphics.cpp
		src/overlay/OverlayGraphics.h)

	target_link_libraries(
		overlay
		arucoUtils
		${OPENGL_LIBRARIES}
		glfw3 X11 Xxf86vm Xrandr Xinerama Xcursor
		)

    target_link_libraries(
            overlay
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)
endif(GLFW3_FOUND)





set(_executables intrinsic_calib extrinsic_calib_aruco robot_to_camera_aruco )
foreach (_executable ${_executables})
    target_link_libraries(
            ${_executable}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)
endforeach ()


