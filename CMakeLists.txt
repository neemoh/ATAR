cmake_minimum_required(VERSION 2.8.3)
project(teleop_vision)

#set(CMAKE_BUILD_TYPE Debug)
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
        sensor_msgs
        tf_conversions
        cv_bridge
        image_transport
        message_generation
        geometry_msgs
        )

add_service_files(
        FILES
        CalculateStereoCamsTransfromFromTopics.srv)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(CATKIN_DEPENDS
        message_runtime
        cv_bridge
        image_transport
        sensor_msgs)




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
        include/utils/Conversions.hpp
)
target_link_libraries(
        arucoUtils
        ${OpenCV_LIBRARIES})


## Build service client and server
# %Tag(SRVCLIENT)%
add_executable(calculate_stereo_cams_transofrm_from_topics
        src/utils/CalculateStereoCamsTransfromFromTopicsServer.cpp)
target_link_libraries(calculate_stereo_cams_transofrm_from_topics
        ${catkin_LIBRARIES})
add_dependencies(calculate_stereo_cams_transofrm_from_topics
        teleop_vision_gencpp)


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
        src/utils/Drawings.cpp
        include/utils/Drawings.hpp
        src/robot_to_camera_aruco/RobotToCameraAruco.hpp)


add_executable(
        extrinsic_calib_aruco
        src/extrinsic_calib_aruco/main_extrinsic_aruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.hpp
        include/utils/BoardDetector.hpp
)
add_library(ExtrinsicCalibArucoNodelet
        src/extrinsic_calib_aruco/ExtrinsicArucoNodelet.cpp
        )

target_link_libraries(ExtrinsicCalibArucoNodelet ${catkin_LIBRARIES} arucoUtils)

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

#find_package(OpenGL)
##find_package(glfw3 CONFIG REQUIRED)
#if(GLFW3_FOUND)
#    MESSAGE("GLFW3 found.")
#    include_directories(${GLFW3_INCLUDE_DIR})
#    #add_executable(
#    #	overlay
#    #	src/overlay/main_overlay.cpp
#    #	src/overlay/OverlayGraphics.cpp
#    #	src/overlay/OverlayGraphics.h)
#
#    target_link_libraries(
#            overlay
#            arucoUtils
#            ${OPENGL_LIBRARIES}
#            glfw3 X11 Xxf86vm Xrandr Xinerama Xcursor
#    )
#endif(GLFW3_FOUND)

add_executable(
        overlay
        src/overlay/main_overlay.cpp
        src/overlay/Overlay.cpp
        src/overlay/Overlay.h
        src/overlay/Drawings.cpp
        src/overlay/Drawings.h
)
target_link_libraries(
        overlay
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        arucoUtils)

add_executable(
        ac_overlay
        src/ac_overlay/main_ac_overlay.cpp
        src/ac_overlay/ACOverlay.cpp
        src/ac_overlay/ACOverlay.h
        src/ac_overlay/Drawings.cpp
        src/ac_overlay/Drawings.h
)
target_link_libraries(
        ac_overlay
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        arucoUtils)


set(_executables intrinsic_calib extrinsic_calib_aruco robot_to_camera_aruco )
foreach (_executable ${_executables})
    target_link_libraries(
            ${_executable}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)
endforeach ()

#add_library(resized_publisher src/resize_transport/manifest.cpp src/resize_transport/resized_publisher.cpp src/resize_transport/resized_subscriber.cpp)
#add_dependencies(resized_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#target_link_libraries(resized_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#install(FILES resized_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

install(TARGETS
        ExtrinsicCalibArucoNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
