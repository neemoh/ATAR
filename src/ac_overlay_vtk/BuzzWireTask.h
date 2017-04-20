//
// Created by nima on 4/18/17.
//

#ifndef TELEOP_VISION_BUZZWIRETASK_H
#define TELEOP_VISION_BUZZWIRETASK_H


#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkImageActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkProperty.h>
#include <vtkParametricTorus.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLocator.h>
#include <vtkTransformPolyDataFilter.h>
#include "Rendering.h"
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>

class BuzzWireTask {
public:

    BuzzWireTask(const double ring_radius, const double wire_radius, const bool show_ref_frames);

    std::vector< vtkSmartPointer <vtkProp> > GetActors();

    void SetCurrentToolPose(const KDL::Frame & tool_pose);

    void UpdateActors();

    void CalculatedDesiredToolPose(const KDL::Frame current_pose,
                                   const KDL::Vector closest_point_to_tool_point,
                                   const KDL::Vector closest_point_to_radial_points,
                                   KDL::Frame &desired_pose);

    KDL::Frame GetDesiredToolPose();


private:

    void FindClosestPoints();
private:

    double ring_radius_;
    double wire_radius_; // the curvy tube from the stl file
    KDL::Vector closest_point_to_tool_point;
    KDL::Vector closest_point_to_radial_point;

    bool show_ref_frames_ = false;

    KDL::Frame tool_desired_pose_kdl;
    KDL::Frame tool_current_pose_kdl;

    vtkSmartPointer<vtkMatrix4x4> tool_current_pose;
//    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose;

    std::vector<vtkSmartPointer<vtkProp>> actors;

    vtkSmartPointer<vtkActor>                       ring_actor;

    vtkSmartPointer<vtkAxesActor>                   task_coordinate_axes;
    vtkSmartPointer<vtkAxesActor>                   tool_current_frame_axes;
    vtkSmartPointer<vtkAxesActor>                   tool_desired_frame_axes;

    vtkSmartPointer<vtkCellLocator>                 cellLocator;

    vtkSmartPointer<vtkLineSource>                  lineSource;
    vtkSmartPointer<vtkPolyDataMapper>              line_mapper;

    vtkSmartPointer<vtkActor>                       ring_guides_mesh_actor;
};
#endif //TELEOP_VISION_BUZZWIRETASK_H
