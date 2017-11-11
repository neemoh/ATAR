//
// Created by nima on 4/18/17.
//

#ifndef TELEOP_VISION_TASKSTEADYHAND_H
#define TELEOP_VISION_TASKSTEADYHAND_H
#include "src/ar_core/SimTask.h"
#include "src/ar_core/SimObject.h"
#include "src/ar_core/Forceps.h"
#include "src/ar_core/Colors.hpp"

#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkPolyData.h>
#include <vtkLineSource.h>
#include <vtkImageActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkTransform.h>
#include <vtkProperty.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLocator.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkCornerAnnotation.h>
#include "src/ar_core/Rendering.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <src/ar_core/Manipulator.h>
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"



/**
 * \class TaskSteadyHand
 * \brief This is a class that generates graphics and logic for a simple ring
 * and wire task, where the user is supposed to move a ring without around a
 * tube without touching it. 10 spheres in the bottom of the image show the
 * score of the last 10 repetitions of the user. The score is based on
 * position error, orientation error and the duration.
 * The error is found from the desired pose of the ring that is calculated from
 * a simple geometric estimation of the closest point on the tube path (for
 * position) and the path tangent on the closest point. This is far from
 * ideal but the goal was to not limit ourselves to parametric curves and be
 * able to use any arbitrary tube.
 * The task has two modes: a simple 1 ring mode, and a bimanual mode where
 * two robot arms are used and each have a ring. TODO: I still have to test
 * this mode.
 *
 * An important point here is that there is a thread in this class that does
 * the spinning for ros and updates the desired pose at a much higher
 * frequency with respect to the 25Hz for graphics which would lead to
 * unstable guidance forces. This is totally not thread-safe! Hopefully in
 * future I learn more about multi-threaded applications and improve this.
 */


enum class SHTaskState: uint8_t {Idle, OnGoing, Finished};


class TaskSteadyHand : public SimTask{
public:

    TaskSteadyHand(ros::NodeHandlePtr n);

    ~TaskSteadyHand();

    // updates the task logic and the graphics_actors
    void TaskLoop() override;

    /**
    * \brief This is the function that is handled by the haptics thread.
    * It first reads the current poses of the tools and then finds the
    * desired pose from the mesh.
    *  **/
    void HapticsThread() override;

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {return graphics_actors;}

    custom_msgs::TaskState GetTaskStateMsg();

    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    void ResetCurrentAcquisition() override;

    // resets the number of repetitions and task state;
    void ResetTask() override;


    // calculates the desired tool pose
    void CalculatedDesiredRingPose(
        KDL::Frame ring_pose,
        KDL::Frame &desired_ring_pose
    );

    // resets the history of the scores and changes the colors to gray
    void ResetScoreHistory();


    // this error is just used to provide feedback to the user. Orientation
    // error is not considered.
    void CalculateAndSaveError();

    // returns the color associated to the score range
    double* GetScoreColor(double score);

    void ResetOnGoingEvaluation();


private:

    // updates the error actor
    void UpdateRingColor();

    // Calculates the closest points of the wire mesh to three points of
    // interest on the ring used for calculating the desired pose of the ring
//    void StepPhysics();

    void UpdateCurrentAndDesiredReferenceFrames(
        const KDL::Frame current_pose[2],
        const KDL::Frame desired_pose[2]
    );

    void UpdateToolRodsPose(
        KDL::Frame pose,
        int gripper_side
    );
private:
    // -------------------------------------------------------------------------
    // task logic

    SHTaskState task_state;
    Colors colors;
    KDL::Frame pose_tube;
    //KDL::Vector idle_point;
    KDL::Vector start_point;
    KDL::Vector end_point;
    custom_msgs::TaskState task_state_msg;
    ros::Time start_time;
    ros::Publisher publisher_task_state;

    double posit_error_sum;
    double orient_error_sum;

    double posit_error_max;
    double orient_error_max;
    uint sample_count;
    std::vector<double> score_history;
    // -------------------------------------------------------------------------
    // graphics
    double ring_radius;
    KDL::Frame ring_pose;
    KDL::Frame estimated_ring_pose;
    KDL::Frame tool_to_ring_tr[2];

    uint ring_in_action = 0;
    bool gripper_in_contact[2] ={false, false};
    bool gripper_in_contact_last[2] ={false, false};
    uint ac_soft_start_counter = 0;
    uint ac_soft_start_duration = 200;

    // the distance between the center of the ring and the closest point on
    // the wire. This is could be slightly different from the error
    // calculated from the difference of the desired pose and the current
    // pose, though not significantly.
    double position_error_norm;
    double orientation_error_norm;
    bool ac_params_changed;

    Manipulator *slaves[2];

    KDL::Frame tool_desired_pose[2];
    KDL::Frame tool_current_pose[2];
    double gripper_angle[2];
    uint destination_ring_counter;

    // graphics_actors that are updated during the task
    vtkSmartPointer<vtkActor>                       destination_ring_actor;
    std::vector< vtkSmartPointer<vtkActor>>         score_sphere_actors;
    std::vector<double*>                            score_history_colors;

    vtkSmartPointer<vtkAxesActor>                   tool_current_frame_axes[2];
    vtkSmartPointer<vtkAxesActor>                   tool_desired_frame_axes[2];

    vtkSmartPointer<vtkCellLocator>                 cellLocator;

    vtkSmartPointer<vtkLineSource>                  line1_source;
    vtkSmartPointer<vtkLineSource>                  line2_source;
    vtkSmartPointer<vtkActor>                       line1_actor;
    vtkSmartPointer<vtkActor>                       line2_actor;

    int ring_num = 4;
    SimObject *ring_mesh[6];
    SimObject *sep_cylinder[6];
    SimObject *tube_meshes[3];
    SimObject *tube_mesh_thin;
    SimObject *stand_mesh;
    SimObject *stand_cube;
    KDL::Vector dir;

    Forceps * forceps[2];
    SimObject* rods[2];
    KDL::Vector rcm[2];

};


#endif //TELEOP_VISION_TaskSteadyHand_H
