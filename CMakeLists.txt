cmake_minimum_required(VERSION 2.8.3)
project(atar)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)

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
        custom_conversions
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
        sensor_msgs
        custom_conversions)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
#find_package(ODE REQUIRED)
find_package(Bullet REQUIRED)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIR}
#        /usr/local/include/bullet/BulletCollision/CollisionShapes/
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

#
#add_executable(intrinsic_calib
#        src/deprecated/intrinsic_calibration.cpp
#        )


add_library(ExtrinsicCalibArucoNodelet
        src/extrinsic_calib_aruco/ExtrinsicArucoNodelet.cpp
        src/extrinsic_calib_aruco/BoardDetector.cpp
        )

target_link_libraries(ExtrinsicCalibArucoNodelet
        ${catkin_LIBRARIES}
        )

add_executable(extrinsic_calib_charuco
        src/extrinsic_calib_aruco/main_extrinsic_charuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.cpp
        src/intrinsic_calib/IntrinsicCalibrationCharuco.h
        )

target_link_libraries(extrinsic_calib_charuco
        ${catkin_LIBRARIES}
        )

#add_executable(calibrate_camera_charuco
#        src/deprecated/intrinsic_calibration_charuco.cpp
#        )
#
#target_link_libraries(calibrate_camera_charuco
#        ${catkin_LIBRARIES} arucoUtils
#        )

add_executable(arm_to_world_calibration
        src/arm_to_world_calibration/main_arm_to_world.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.h
        )

target_link_libraries(arm_to_world_calibration
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )


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





##########################################################################
#                           Build Common Nodes
##########################################################################


add_executable(create_aruco_board
        src/utils/create_aruco_board.cpp
        )

add_executable(create_charuco_board
        src/utils/create_charuco_board.cpp
        )

set(aruco_executables create_aruco_board create_charuco_board)
foreach (_ex ${aruco_executables})
    target_link_libraries(
            ${_ex}
            ${OpenCV_LIBRARIES}
            )
endforeach ()


add_executable(
        stereo_image_view
        src/stereo_image_view/main_stereo_image_view.cpp
)

add_executable(
        stereo_image_split
        src/stereo_image_view/main_stereo_image_split_TEMPORARY.cpp
)

add_executable(stereo_usb_cam_publisher
        src/stereo_usb_cam_publisher/main_stereo_usb_cam_publisher.cpp
        )

add_executable(
        ac_overlay_cv
        src/ac_overlay_cv/main_ac_overlay.cpp
        src/ac_overlay_cv/ACOverlay.cpp
        src/ac_overlay_cv/ACOverlay.h
        src/ac_overlay_cv/Drawings.cpp
        src/ac_overlay_cv/Drawings.h
)

set(executables
        stereo_image_view
        stereo_image_split
        stereo_usb_cam_publisher
        ac_overlay_cv)

foreach (_ex ${executables})
    target_link_libraries(
            ${_ex}
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES})
endforeach ()


add_executable(bullet_test src/bullet_test.cpp )
target_link_libraries(bullet_test  BulletDynamics BulletCollision LinearMath
        )

##########################################################################
#                           AC OVERLAY VTK
##########################################################################

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

add_library(LoadObjGL
        src/ac_overlay_vtk/LoadObjGL/LoadMeshFromObj.h
        src/ac_overlay_vtk/LoadObjGL/LoadMeshFromObj.cpp
        src/ac_overlay_vtk/LoadObjGL/Wavefront2GLInstanceGraphicsShape.cpp
        src/ac_overlay_vtk/LoadObjGL/Wavefront2GLInstanceGraphicsShape.cpp
        src/ac_overlay_vtk/LoadObjGL/tiny_obj_loader.cpp
        src/ac_overlay_vtk/LoadObjGL/tiny_obj_loader.cpp
        src/ac_overlay_vtk/LoadObjGL/GLInstanceGraphicsShape.h
#        src/ac_overlay_vtk/LoadObjGL/GLInstancingRenderer.cpp
#        src/ac_overlay_vtk/LoadObjGL/GLInstancingRenderer.h
#        src/ac_overlay_vtk/LoadObjGL/OpenGLInclude.h

        )

target_link_libraries(
        LoadObjGL
        BulletDynamics
        BulletCollision
        LinearMath
        Bullet3Common
)


add_executable(
        ac_overlay_vtk
        src/ac_overlay_vtk/main_ac_overlay_vtk.cpp
        src/ac_overlay_vtk/CalibratedCamera.cpp
        src/ac_overlay_vtk/CalibratedCamera.h
        src/ac_overlay_vtk/Rendering.cpp
        src/ac_overlay_vtk/Rendering.h
        src/ac_overlay_vtk/OverlayROSConfig.cpp
        src/ac_overlay_vtk/OverlayROSConfig.h
        src/ac_overlay_vtk/VTKTask.h
        src/ac_overlay_vtk/TaskBuzzWire.cpp
        src/ac_overlay_vtk/TaskBuzzWire.h
        src/ac_overlay_vtk/TaskKidney.cpp
        src/ac_overlay_vtk/TaskKidney.h
        src/arm_to_world_calibration/ArmToWorldCalibration.cpp
        src/arm_to_world_calibration/ArmToWorldCalibration.h
        src/ac_overlay_vtk/TaskODE.cpp
        src/ac_overlay_vtk/TaskODE.h
        src/ac_overlay_vtk/TaskBullet.cpp
        src/ac_overlay_vtk/TaskBullet.h
        src/ac_overlay_vtk/TaskBulletTest.cpp
        src/ac_overlay_vtk/TaskBulletTest.h
        src/ac_overlay_vtk/BulletVTKMotionState.h
        src/ac_overlay_vtk/BulletVTKObject.cpp
        src/ac_overlay_vtk/BulletVTKObject.h
        src/ac_overlay_vtk/ControlEvents.h)


target_link_libraries(
        ac_overlay_vtk
        ${OpenCV_LIBRARIES}
        ${VTK_LIBRARIES}
        ${catkin_LIBRARIES}
        BulletDynamics
        BulletCollision
        LinearMath
        libode.a
        LoadObjGL
        )
set(ODE_PRECISION dSINGLE)

add_executable(
        vtk_shadow_test src/vtkshadowtest.cpp
)

target_link_libraries(
        vtk_shadow_test
        ${OpenCV_LIBRARIES}
        ${VTK_LIBRARIES}
)
##########################################################################
#                           Reporter node
##########################################################################

find_package(Qt5Widgets)
find_package(Qt5Core REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(PROJECT_INCLUDE_DIR src/gui/include)

set (MyProjectLib_hdr
        ${PROJECT_INCLUDE_DIR}/mainwindow.hpp
        ${PROJECT_INCLUDE_DIR}/cvimagewidget.h)
set (MyProjectLib_src
        src/gui/mainwindow.cpp
        src/gui/rosclass.cpp)
qt5_wrap_cpp(MyProjectLib_hdr_moc ${MyProjectLib_hdr})
qt5_wrap_ui (MyProjectLib_ui_moc src/gui/mainwindow.ui)

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})
include_directories (${PROJECT_INCLUDE_DIR})

add_library (reporter_utils SHARED
        ${MyProjectLib_src}
        ${MyProjectLib_hdr_moc}
        ${MyProjectLib_ui_moc}
        )

target_link_libraries (reporter_utils
        Qt5::Widgets
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES})

add_executable(reporter src/gui/ui_node.cpp)
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
