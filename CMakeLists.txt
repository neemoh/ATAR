cmake_minimum_required(VERSION 2.8.3)
project(atar)

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
        custom_msgs
        active_constraints
        )

#find_package(OpenCV 3 REQUIRED)
#add_service_files(
 #       FILES
  #      CalculateStereoCamsTransfromFromTopics.srv)

#add_message_files(
#        FILES
#        TaskState.msg
#)

#generate_messages(
#        DEPENDENCIES
#        geometry_msgs
#)

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
#add_executable(calculate_stereo_cams_transform_from_topics
#        src/utils/CalculateStereoCamsTransfromFromTopicsServer.cpp
#        )
#target_link_libraries(calculate_stereo_cams_transform_from_topics
#        ${catkin_LIBRARIES}
 #       )
#add_dependencies(calculate_stereo_cams_transform_from_topics
#        teleop_vision_gencpp
 #       )


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


add_executable(extrinsic_calib_charuco
        src/extrinsic_calib_aruco/main_extrinsic_charuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.h
        )

target_link_libraries(extrinsic_calib_charuco
        arucoUtils
        ${catkin_LIBRARIES}
        )

add_executable(calibrate_camera_charuco
        src/intrinsic_calib/intrinsic_calibration_charuco.cpp
        )

target_link_libraries(calibrate_camera_charuco
        ${catkin_LIBRARIES} arucoUtils
        )


add_executable(stereo_usb_cam_publisher
        src/stereo_usb_cam_publisher/main_stereo_usb_cam_publisher.cpp
        )

target_link_libraries(stereo_usb_cam_publisher
        ${catkin_LIBRARIES}
        )


##########################################################################
#                           FlyCapture SDK
##########################################################################

option(WITH_FlyCapture "Enable support for the FlyCapture SDK" OFF)
#
#if (WITH_FlyCapture)
#    # Find the Flycapture include files and libraries
#    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/src/flycapture)
#
#    find_package(Flycapture2)
#    if (FLYCAPTURE2_FOUND)
#        MESSAGE("FLYCAPTURE2 found.")
#        include_directories(${FLYCAPTURE2_INCLUDE_DIR})
#
#        add_executable(camera_capture_flea3
#                src/camera_capture_flea3/main_camera_capture_flea3.cpp
#                src/camera_capture_flea3/Fla3Camera.cpp
#                src/camera_capture_flea3/Fla3Camera.h
#                )
#        target_link_libraries(camera_capture_flea3
#                ${FLYCAPTURE2_LIBRARIES}
#                )
#        target_link_libraries(camera_capture_flea3
#                ${OpenCV_LIBRARIES}
#                ${catkin_LIBRARIES}
#                arucoUtils
#                )
#    endif (FLYCAPTURE2_FOUND)
#endif (WITH_FlyCapture)

##########################################################################
#                           Build Common Nodes
##########################################################################


add_executable(create_aruco_board
        src/utils/create_aruco_board.cpp
        )
target_link_libraries(create_aruco_board
        ${OpenCV_LIBRARIES}
        )

add_executable(create_charuco_board
        src/utils/create_charuco_board.cpp
        )
target_link_libraries(create_charuco_board
        ${OpenCV_LIBRARIES}
        )

add_executable(detect_board_charuco
        src/utils/detect_board_charuco.cpp
        )
target_link_libraries(detect_board_charuco
        ${OpenCV_LIBRARIES}
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
        stereo_image_split
        src/stereo_image_view/main_stereo_image_split_TEMPORARY.cpp
)
target_link_libraries(
        stereo_image_split
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
)



set(_executables intrinsic_calib extrinsic_calib_aruco robot_to_camera_aruco )
foreach (_executable ${_executables})
    target_link_libraries(
            ${_executable}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            arucoUtils)
endforeach ()
##########################################################################
#                           AC OVERLAY
##########################################################################



add_executable(
        ac_overlay_cv
        src/ac_overlay_cv/main_ac_overlay.cpp
        src/ac_overlay_cv/ACOverlay.cpp
        src/ac_overlay_cv/ACOverlay.h
        src/utils/Drawings.cpp
        include/utils/Drawings.h
)

target_link_libraries(
        ac_overlay_cv
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        arucoUtils)

##########################################################################
#                           AC OVERLAY VTK

if(POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW) # CMake 3.0
endif()
if(POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW) # CMake 3.1
endif()

find_package(VTK REQUIRED)
vtk_module_config(VTK
        vtkCommonCore
        vtkCommonDataModel
        vtkFiltersCore
        vtkFiltersModeling
        vtkIOXML
        vtkInteractionStyle
        vtkRendering${VTK_RENDERING_BACKEND}
        vtkImagingGeneral
        vtkInteractionWidgets
        vtkIOGeometry
        )

include(${VTK_USE_FILE})
add_executable(
        ac_overlay_vtk
        src/ac_overlay_vtk/main_ac_overlay_vtk.cpp
        src/ac_overlay_vtk/CalibratedCamera.cpp
        src/ac_overlay_vtk/CalibratedCamera.h
        src/ac_overlay_vtk/Rendering.cpp
        src/ac_overlay_vtk/Rendering.h
        src/utils/Drawings.cpp
        include/utils/Drawings.h
        src/ac_overlay_vtk/OverlayROSConfig.cpp
        src/ac_overlay_vtk/OverlayROSConfig.h
        src/ac_overlay_vtk/VTKTask.h
        src/ac_overlay_vtk/BuzzWireTask.cpp
        src/ac_overlay_vtk/BuzzWireTask.h
        src/ac_overlay_vtk/KidneyTask.cpp
        src/ac_overlay_vtk/KidneyTask.h
)

target_link_libraries(
        ac_overlay_vtk
        ${OpenCV_LIBRARIES}
        ${VTK_LIBRARIES}
        ${catkin_LIBRARIES}
        arucoUtils)



##########################################################################
#                           AC OVERLAY VTK

find_package(Qt5Widgets)
find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(PROJECT_INCLUDE_DIR include)

set (MyProjectLib_hdr
        src/reporter/include/mainwindow.hpp
        src/reporter/include/cvimagewidget.h)
set (MyProjectLib_src
        src/reporter/mainwindow.cpp
        src/reporter/rosclass.cpp)
qt5_wrap_cpp(MyProjectLib_hdr_moc ${MyProjectLib_hdr})
qt5_wrap_ui (MyProjectLib_ui_moc  src/reporter/mainwindow.ui)

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})
include_directories (src/reporter/include)

add_library (reporter_utils SHARED
        ${MyProjectLib_src}
        ${MyProjectLib_hdr_moc}
        ${MyProjectLib_ui_moc}
        )

target_link_libraries (reporter_utils
        Qt5::Widgets
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(reporter src/reporter/labelling_node.cpp)
target_link_libraries (reporter
        ${catkin_LIBRARIES}
        reporter_utils
        )


##########################################################################
#                           Install Targets
##########################################################################

install(TARGETS
        ExtrinsicCalibArucoNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
