//
// Created by nima on 4/18/17.
//

#ifndef TELEOP_VISION_TASKSTEADYHAND_H
#define TELEOP_VISION_TASKSTEADYHAND_H
#include "VTKTask.h"

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
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPolyDataNormals.h>
#include <vtkCornerAnnotation.h>

#include "Rendering.h"
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <btBulletDynamicsCommon.h>
#include "BulletVTKObject.h"
#include "Forceps.h"
#include "Colors.hpp"

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
 * able to use any aribitrary tube.
 * The task has two modes: a simple 1 ring mode, and a bimanual mode where
 * two robot arms are used and each have a ring. TODO: I still have to test
 * this mode.
 *
 * An important point here is that there is a thread in this class that doeas
 * the spinning for ros and updates the desired pose at a much higher
 * frequency with respect to the 25Hz for graphics which would lead to
 * unstable guidance forces. This is totally not thread-safe! Hopefully in
 * future I learn more about multi-threaded applications and improve this.
 */


enum class SHTaskState: uint8_t {Idle, OnGoing, Finished};


class TaskSteadyHand : public VTKTask{
public:

    TaskSteadyHand(
            const std::string mesh_file_dir,
            const bool show_ref_frames,
            const bool num_tools,
            const bool with_guidance,
            const double haptic_loop_rate,
            const std::string slave_names[],
            KDL::Frame *slave_to_world_tr
        );

    ~TaskSteadyHand();
    // returns all the task actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return actors;
    }
    // sets the pose of the tools
    void SetCurrentToolPosePointer(KDL::Frame &tool_pose, const int tool_id);


    // sets the position of the gripper
    void SetCurrentGripperpositionPointer(double &gripper_position, const int
    tool_id);

    // updates the task logic and the actors
    void UpdateActors();

    // calculates the desired tool pose
    void CalculatedDesiredToolPose(const KDL::Frame ring_pose,
                                   KDL::Frame &desired_tool_pose);

    // returns the status of the change of the ac_param
    bool IsACParamChanged();

    // returns the ac parameters
    custom_msgs::ActiveConstraintParameters* GetACParameters();

    custom_msgs::TaskState GetTaskStateMsg();

    // resets the number of repetitions and task state;
    void ResetTask();

    // resets the history of the scores and changes the colors to gray
    void ResetScoreHistory();

    // decrements the number of repetitions. Used in case something goes
    // wrong during that repetition.
    void ResetCurrentAcquisition();

    // this error is just used to provide feedback to the user. Orientation
    // error is not considered.
    void CalculateAndSaveError();

    // returns the color associated to the score range
    double* GetScoreColor(const double score);

    void ResetOnGoingEvaluation();

    /**
    * \brief This is the function that is handled by the desired pose thread.
     * It first reads the current poses of the tools and then finds the
     * desired pose from the mesh.
  *  **/
    void FindAndPublishDesiredToolPose();

private:

    // updates the error actor
    void UpdateTubeColor();

    // Calculates the closest points of the wire mesh to three points of
    // interest on the ring used for calculating the desired pose of the ring

    void InitBullet();

    void StepDynamicsWorld();

    void UpdateCurrentAndDesiredReferenceFrames(
        const KDL::Frame current_pose[2],
        const KDL::Frame desired_pose[2]
    );

    void UpdateToolRodsPose(
        const KDL::Frame pose,
        int gripper_side
    );
private:

    // -------------------------------------------------------------------------
    // task logic
//    bool bimanual;
//    bool with_guidance;
    SHTaskState task_state;
    std::string mesh_files_dir;
    std::string *slave_names;
    KDL::Frame *slave_frame_to_world_frame_tr;
    Colors colors;

    KDL::Vector idle_point;
    KDL::Vector start_point;
    KDL::Vector end_point;
    custom_msgs::TaskState task_state_msg;
    ros::Time start_time;
    double posit_error_sum;
    double orient_error_sum;

    double posit_error_max;
    uint sample_count;
    uint n_score_history;
    std::vector<double> score_history;
    // -------------------------------------------------------------------------
    // graphics
    double ring_radius;
    KDL::Frame ring_pose;
    KDL::Frame tool_to_ring_tr[2];

    uint ring_in_action = 0;
    bool gripper_in_contact[2] ={false, false};
    // the distance between the center of the ring and the closest point on
    // the wire. This is could be slightly different from the error
    // calculated from the difference of the desired pose and the current
    // pose, though not significantly.
    double position_error_norm;
    double orientation_error_norm;

    bool ac_params_changed;
    custom_msgs::ActiveConstraintParameters ac_parameters[2];

    KDL::Frame tool_desired_pose[2];
    KDL::Frame *tool_current_pose_ptr[2];
    KDL::Frame tool_current_pose[2];
//    KDL::Frame tool_last_pose[2];

    uint destination_ring_counter;
//    vtkSmartPointer<vtkMatrix4x4> tool_current_pose[2];


    // actors that are updated during the task
    vtkSmartPointer<vtkActor>                       destination_ring_actor;
    std::vector< vtkSmartPointer<vtkActor>>         score_sphere_actors;
    std::vector<double*>                           score_history_colors;

    vtkSmartPointer<vtkAxesActor>                   tool_current_frame_axes[2];
    vtkSmartPointer<vtkAxesActor>                   tool_desired_frame_axes[2];

    vtkSmartPointer<vtkCellLocator>                 cellLocator;

    vtkSmartPointer<vtkLineSource>                  line1_source;
    vtkSmartPointer<vtkLineSource>                  line2_source;
    vtkSmartPointer<vtkActor>                       line1_actor;
    vtkSmartPointer<vtkActor>                       line2_actor;

    vtkSmartPointer<vtkCornerAnnotation>            cornerAnnotation;

    // dynamics

    ros::Time time_last;
    btDiscreteDynamicsWorld* dynamics_world;
    btSequentialImpulseConstraintSolver* solver;
    btBroadphaseInterface* overlappingPairCache;
    btCollisionDispatcher* dispatcher;
    btDefaultCollisionConfiguration* collisionConfiguration;

    int ring_num = 6;
    BulletVTKObject *ring_mesh[6];
    BulletVTKObject *sep_cylinder[6];
    BulletVTKObject *tube_meshes[3];
    BulletVTKObject *tube_mesh_thin;
    BulletVTKObject *stand_mesh;
    BulletVTKObject *stand_cube;
    KDL::Vector dir;
    // ADDED
    BulletVTKObject *trans_cyl;
    // -----

//    std::vector<std::vector<double>> gripper_link_dims;
    Forceps * forceps[2];

    BulletVTKObject* closing_cylinder;
    BulletVTKObject* arm[2];
    KDL::Vector rcm[2];

    double * gripper_position[2];


};


#endif //TELEOP_VISION_TaskSteadyHand_H
