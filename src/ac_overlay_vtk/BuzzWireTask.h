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
#include <vtkCornerAnnotation.h>

#include "active_constraints/ActiveConstraintParameters.h"

class BuzzWireTask {
public:

    BuzzWireTask(const double ring_radius, const bool show_ref_frames);

    std::vector< vtkSmartPointer <vtkProp> > GetActors();

    void SetCurrentToolPose(const KDL::Frame & tool_pose);

    void UpdateActors();

    void CalculatedDesiredToolPose(const KDL::Frame current_pose,
                                       const KDL::Vector closest_point_to_ring_center,
                                       const KDL::Vector closest_point_to_radial_point,
                                       const KDL::Vector closest_point_to_grip_point,
                                       KDL::Frame &desired_pose);

    KDL::Frame GetDesiredToolPose();

    bool IsACParamChanged();

    active_constraints::ActiveConstraintParameters GetACParameters();


private:

    void FindClosestPoints();
private:

    double ring_radius_;
//    double wire_radius_; // the curvy tube from the stl file
    KDL::Vector closest_point_to_ring_center;
    KDL::Vector closest_point_to_radial_point;
    KDL::Vector closest_point_to_grip_point;

    double error_position;

    bool show_ref_frames_ = false;

    bool ac_params_changed = true; // to publish once at the begging
    active_constraints::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl;
    KDL::Frame tool_current_pose_kdl;

    uint destination_cone_counter = 0;
    // graphics
    vtkSmartPointer<vtkMatrix4x4> tool_current_pose;
//    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose;

    std::vector<vtkSmartPointer<vtkProp>> actors;

    vtkSmartPointer<vtkActor>                       ring_actor;
    vtkSmartPointer<vtkActor>                       error_sphere_actor;

    vtkSmartPointer<vtkAxesActor>                   task_coordinate_axes;
    vtkSmartPointer<vtkAxesActor>                   tool_current_frame_axes;
    vtkSmartPointer<vtkAxesActor>                   tool_desired_frame_axes;

    vtkSmartPointer<vtkCellLocator>                 cellLocator;

    vtkSmartPointer<vtkLineSource>                  line1_source;
    vtkSmartPointer<vtkLineSource>                  line2_source;

    vtkSmartPointer<vtkActor>                       ring_guides_mesh_actor;

    vtkSmartPointer<vtkActor>                       destination_cone_actor;

    vtkSmartPointer<vtkCornerAnnotation>            cornerAnnotation;
};
#endif //TELEOP_VISION_BUZZWIRETASK_H
