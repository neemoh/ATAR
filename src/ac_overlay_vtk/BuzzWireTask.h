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
#include "teleop_vision/TaskState.h"

// P1 is the point where the ring enters the wire
// P2 is the destination point shown to the user.
// The user goes from the start point to the end point and back.
enum TaskState: uint8_t {Idle, ToEndPoint, ToStartPoint, RepetitionComplete};


class BuzzWireTask {
public:

    BuzzWireTask(const double ring_radius, const bool show_ref_frames);

    // returns all the task actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors();

    // sets the pose of the tools
    void SetCurrentToolPose(const KDL::Frame & tool_pose);

    // updates the task logic and the actors
    void UpdateActors();

    // calculates the desired tool pose
    void CalculatedDesiredToolPose();

    // calculates and returns the desired pose.
    // It called faster than the graphic loop rate
    KDL::Frame GetDesiredToolPose();

    // returns the status of the change of the ac_param
    bool IsACParamChanged();

    // returns the ac parameters
    active_constraints::ActiveConstraintParameters GetACParameters();

    teleop_vision::TaskState GetTaskStateMsg();

    // resets the number of repetitions and task state;
    void Reset();

    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    void RepeatLastAcquisition();

private:

    // updates the error actor
    void UpdatePositionErrorActor();

    // Calculates the closest points of the wire mesh to three points of
    // interest on the ring used for calculating the desired pose of the ring
    void FindClosestPoints();

private:

    // -------------------------------------------------------------------------
    // task logic
    TaskState task_state;

    KDL::Vector idle_point;
    KDL::Vector start_point;
    KDL::Vector end_point;
    teleop_vision::TaskState task_state_msg;
    uint8_t number_of_repetition;
    ros::Time start_time;

    // -------------------------------------------------------------------------
    // graphics
    double ring_radius_;
    KDL::Vector ring_center;
    KDL::Vector closest_point_to_ring_center;
    KDL::Vector closest_point_to_radial_point;
    KDL::Vector closest_point_to_grip_point;

    // the distance between the center of the ring and the closest point on
    // the wire. This is could be slightly different from the error
    // calculated from the difference of the desired pose and the current
    // pose, though not significantly.
    double position_error_norm;

    bool show_ref_frames;

    bool ac_params_changed;
    active_constraints::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl;
    KDL::Frame tool_current_pose_kdl;

    uint destination_cone_counter;
    vtkSmartPointer<vtkMatrix4x4> tool_current_pose;

    std::vector<vtkSmartPointer<vtkProp>>           actors;

    // actors that are updated during the task
    vtkSmartPointer<vtkActor>                       ring_actor;
    vtkSmartPointer<vtkActor>                       error_sphere_actor;

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
