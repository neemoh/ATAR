cmake_minimum_required(VERSION 2.8.3)
project(atar)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

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
        #        message_generation
        geometry_msgs
        custom_msgs
        custom_conversions
        active_constraints)

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
        sensor_msgs
        custom_conversions)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIR})

SUBDIRS( src/ar_core/LoadObjGL/VHACD)

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

#
#add_executable(intrinsic_calib
#        src/deprecated/intrinsic_calibration.cpp
#        )


##########################################################################
#                    Build Calibration nodes
##########################################################################

add_executable(extrinsic_calib_charuco
        src/extrinsic_calib_aruco/main_extrinsic_charuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.h)

target_link_libraries(extrinsic_calib_charuco
        ${catkin_LIBRARIES})

add_executable(create_aruco_board
        src/utils/create_aruco_board.cpp)

add_executable(create_charuco_board
        src/utils/create_charuco_board.cpp)

set(aruco_executables create_aruco_board create_charuco_board)
foreach (_ex ${aruco_executables})
    target_link_libraries(
            ${_ex}
            ${OpenCV_LIBRARIES}
    )
endforeach ()

add_executable(arm_to_world_calibration
        src/arm_to_world_calibration/main_arm_to_world.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.h)

target_link_libraries(arm_to_world_calibration
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_library(ExtrinsicCalibArucoNodelet
        src/extrinsic_calib_aruco/ExtrinsicArucoNodelet.cpp
        src/extrinsic_calib_aruco/BoardDetector.cpp)

target_link_libraries(ExtrinsicCalibArucoNodelet
        ${catkin_LIBRARIES})

##########################################################################
#                           Build Common Nodes
##########################################################################
add_executable(
        stereo_image_view
        src/stereo_image_view/main_stereo_image_view.cpp)

add_executable(
        stereo_image_split
        src/stereo_image_view/main_stereo_image_split_TEMPORARY.cpp)

add_executable(stereo_usb_cam_publisher
        src/stereo_usb_cam_publisher/main_stereo_usb_cam_publisher.cpp)

add_executable(
        teleop_dummy_dvrk
        src/teleop_dummy/main_teleop_dummy_dvrk.cpp)

#add_executable(
#        teleop_dummy_sigma
#        src/teleop_dummy/main_teleop_dummy_sigma.cpp)

set(executables
        stereo_image_view
        stereo_image_split
        stereo_usb_cam_publisher
        teleop_dummy_dvrk
        #        teleop_dummy_sigma
        )

foreach (_ex ${executables})
    target_link_libraries(
            ${_ex}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES})
endforeach ()


##########################################################################
#                               AR CORE
##########################################################################

if(POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW) # CMake 3.0
endif()
if(POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW) # CMake 3.1
endif()
SET(GCC_COVERAGE_COMPILE_FLAGS "-fopenmp")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )

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
        vtkIOGeometry)

include(${VTK_USE_FILE})

add_library(LoadObjGL
        src/ar_core/LoadObjGL/LoadMeshFromObj.h
        src/ar_core/LoadObjGL/LoadMeshFromObj.cpp
        src/ar_core/LoadObjGL/Wavefront2GLInstanceGraphicsShape.cpp
        src/ar_core/LoadObjGL/Wavefront2GLInstanceGraphicsShape.cpp
        src/ar_core/LoadObjGL/tiny_obj_loader.cpp
        src/ar_core/LoadObjGL/tiny_obj_loader.cpp
        src/ar_core/LoadObjGL/GLInstanceGraphicsShape.h
        src/ar_core/LoadObjGL/VHACDGen.cpp
        src/ar_core/LoadObjGL/VHACDGen.h)

target_link_libraries(
        LoadObjGL
        VHACD_local
        BulletDynamics
        BulletCollision
        LinearMath
        Bullet3Common)

# Assume all files that start with "Task" are to be added
file(GLOB tasks_src "src/ar_core/tasks/Task*.cpp")
file(GLOB tasks_h "src/ar_core/tasks/Task*.h")

add_executable(
        ar_core
        src/ar_core/main_ar.cpp
        src/ar_core/RenderingCamera.cpp
        src/ar_core/RenderingCamera.h
        src/ar_core/Rendering.cpp
        src/ar_core/Rendering.h
        src/ar_core/TaskHandler.cpp
        src/ar_core/TaskHandler.h
        src/arm_to_world_calibration/ArmToWorldCalibration.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.h
        src/ar_core/ControlEvents.h
        src/ar_core/BulletVTKMotionState.h
        src/ar_core/SimObject.cpp
        src/ar_core/SimObject.h
        src/ar_core/SimTask.h
        ${tasks_src}
        ${tasks_h}
        src/ar_core/SimSoftObject.cpp
        src/ar_core/SimSoftObject.h
        src/ar_core/FiveLinkGripper.cpp
        src/ar_core/FiveLinkGripper.h
        src/ar_core/Forceps.cpp
        src/ar_core/Forceps.h
        src/ar_core/Colors.hpp
        src/ar_core/VTKConversions.h
        src/ar_core/VTKConversions.cpp
        src/ar_core/Manipulator.cpp
        src/ar_core/Manipulator.h
        src/ar_core/AugmentedCamera.cpp src/ar_core/AugmentedCamera.h src/ar_core/SimTask.cpp)

target_link_libraries(
        ar_core
        ${OpenCV_LIBRARIES}
        ${VTK_LIBRARIES}
        ${catkin_LIBRARIES}
        BulletDynamics
        BulletCollision
        LinearMath
        LoadObjGL
        BulletSoftBody
        pthread)

##########################################################################
#                           GUI node
##########################################################################

find_package(Qt5Widgets)
find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(PROJECT_INCLUDE_DIR src/gui/include)

set (gui_hdr
        ${PROJECT_INCLUDE_DIR}/mainwindow.hpp
        ${PROJECT_INCLUDE_DIR}/cvimagewidget.h)
set (gui_src
        src/gui/mainwindow.cpp
        src/gui/rosclass.cpp
        src/gui/PerformanceEvaluators.h
        src/gui/PerformanceEvaluators.cpp)

qt5_wrap_cpp(gui_hdr_moc ${gui_hdr})
qt5_wrap_ui (gui_ui_moc src/gui/mainwindow.ui)

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})
include_directories (${PROJECT_INCLUDE_DIR})

add_library (gui_libs SHARED
        ${gui_src}
        ${gui_hdr_moc}
        ${gui_ui_moc})

target_link_libraries (gui_libs
        Qt5::Widgets
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(gui src/gui/ui_node.cpp)
target_link_libraries (gui
        ${catkin_LIBRARIES}
        gui_libs)


##########################################################################
#                           Install Targets
##########################################################################

install(TARGETS
        ExtrinsicCalibArucoNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




##########################################################################
#                           FlyCapture SDK
##########################################################################

#option(WITH_FlyCapture "Enable support for the FlyCapture SDK" OFF)
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

