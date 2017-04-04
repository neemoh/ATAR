cmake_minimum_required(VERSION 2.8.3)
project(teleop_vision)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)

option(WITH_ROS "Enable support for ROS topics in opencv_cam_calib" OFF)
if (WITH_ROS)
    add_definitions(-DWITH_ROS)
endif ()

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

include_directories(
        ${catkin_INCLUDE_DIRS}
        include/
        )

##########################################################################
#                           ArucoUtils Library
##########################################################################


add_library(arucoUtils
        src/utils/BoardDetector.cpp
        src/utils/Conversions.cpp
        include/utils/Conversions.hpp
        )
target_link_libraries(arucoUtils
        ${OpenCV_LIBRARIES}
        )


##########################################################################
#                    Build Client and Service Nodes
##########################################################################

# %Tag(SRVCLIENT)%
add_executable(calculate_stereo_cams_transform_from_topics
        src/utils/CalculateStereoCamsTransfromFromTopicsServer.cpp
        )
target_link_libraries(calculate_stereo_cams_transform_from_topics
        ${catkin_LIBRARIES}
        )
add_dependencies(calculate_stereo_cams_transform_from_topics
        teleop_vision_gencpp
        )

add_executable(intrinsic_calib
        src/intrinsic_calib/intrinsic_calibration.cpp
        )

add_executable(robot_to_camera_aruco
        src/robot_to_camera_aruco/main_robot_to_camera_aruco.cpp
        src/robot_to_camera_aruco/RobotToCameraAruco.cpp
        src/utils/Drawings.cpp
        include/utils/Drawings.h
        src/robot_to_camera_aruco/RobotToCameraAruco.hpp
        )


add_executable(extrinsic_calib_aruco
        src/extrinsic_calib_aruco/main_extrinsic_aruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.cpp
        src/extrinsic_calib_aruco/ExtrinsicAruco.hpp
        include/utils/BoardDetector.hpp
        )
target_link_libraries(extrinsic_calib_aruco
        ${OpenCV_LIBRARIES}
)

add_library(ExtrinsicCalibArucoNodelet
        src/extrinsic_calib_aruco/ExtrinsicArucoNodelet.cpp
        )

target_link_libraries(ExtrinsicCalibArucoNodelet
        ${catkin_LIBRARIES} arucoUtils
        )

##########################################################################
#                           FlyCapture SDK
##########################################################################

option(WITH_FlyCapture "Enable support for the FlyCapture SDK" OFF)

if (WITH_FlyCapture)
    # Find the Flycapture include files and libraries
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/src/flycapture)

    find_package(Flycapture2)
    if (FLYCAPTURE2_FOUND)
        MESSAGE("FLYCAPTURE2 found.")
        include_directories(${FLYCAPTURE2_INCLUDE_DIR})

        add_executable(camera_capture_flea3
                src/camera_capture_flea3/main_camera_capture_flea3.cpp
                src/camera_capture_flea3/Fla3Camera.cpp
                src/camera_capture_flea3/Fla3Camera.h
                )
        target_link_libraries(camera_capture_flea3
                ${FLYCAPTURE2_LIBRARIES}
                )
        target_link_libraries(camera_capture_flea3
                ${OpenCV_LIBRARIES}
                ${catkin_LIBRARIES}
                arucoUtils
                )
    endif (FLYCAPTURE2_FOUND)
endif (WITH_FlyCapture)

##########################################################################
#                           Build Common Nodes
##########################################################################


add_executable(create_aruco_board
        src/utils/create_aruco_board.cpp
        )
target_link_libraries(create_aruco_board
        ${OpenCV_LIBRARIES}
        )

add_executable(overlay
        src/overlay/main_overlay.cpp
        src/overlay/Overlay.cpp
        src/overlay/Overlay.h
        src/utils/Drawings.cpp
        include/utils/Drawings.h
        )
target_link_libraries(overlay
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        arucoUtils
        )

add_executable(
        stereo_image_view
        src/stereo_image_view/main_stereo_image_view.cpp
)
target_link_libraries(
        stereo_image_view
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)


add_executable(
        ac_overlay
        src/ac_overlay/main_ac_overlay.cpp
        src/ac_overlay/ACOverlay.cpp
        src/ac_overlay/ACOverlay.h
        src/utils/Drawings.cpp
        include/utils/Drawings.h
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

##########################################################################
#                           Install Targets
##########################################################################

install(TARGETS
        ExtrinsicCalibArucoNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
