//
// Created by nima on 4/18/17.
//

#ifndef TELEOP_VISION_TASKKIDNEY_H
#define TELEOP_VISION_TASKKIDNEY_H
#include "src/ar_core/SimTask.h"

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

#include "src/ar_core/Rendering.h"
#include "custom_msgs/ActiveConstraintParameters.h"
#include "custom_msgs/TaskState.h"
#include <ros/ros.h>
#include <std_msgs/Empty.h>

/**
 * \class TaskKidney
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


enum class TaskKidneyState: uint8_t {Idle, ToEndPoint, ToStartPoint,
    RepetitionComplete};



// -------------------------------------------------------------------------


class TaskKidney : public SimTask{
public:

    TaskKidney(const std::string stl_files_dir,
                     const bool show_ref_frames, const bool num_tools,
                     const bool with_guidance);

//    ~TaskKidney();

    // returns all the task graphics_actors to be sent to the rendering part
    std::vector< vtkSmartPointer <vtkProp> > GetActors() {
        return graphics_actors;
    }

    // updates the task logic and the graphics_actors
    void StepWorld();

    // calculates the desired tool pose
    void CalculatedDesiredToolPose();

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
    void HapticsThread();

private:

    // updates the error actor
    void UpdateTubeColor();

    // Calculates the closest points of the wire mesh to three points of
    // interest on the ring used for calculating the desired pose of the ring

public:

private:


    // -------------------------------------------------------------------------
    // task logic
    bool bimanual;
    bool with_guidance;
    TaskKidneyState task_state;
    std::string stl_files_dir;

    KDL::Vector idle_point;
    KDL::Vector start_point;
    KDL::Vector end_point;
    custom_msgs::TaskState task_state_msg;
    uint8_t number_of_repetition;
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
    KDL::Vector ring_center[2];
    KDL::Vector closest_point_to_ring_center[2];
    KDL::Vector closest_point_to_radial_point[2];
    KDL::Vector closest_point_to_grip_point[2];

    // the distance between the center of the ring and the closest point on
    // the wire. This is could be slightly different from the error
    // calculated from the difference of the desired pose and the current
    // pose, though not significantly.
    double position_error_norm[2];
    double orientation_error_norm[2];

//    bool show_ref_frames;

    // for not we use the same type of active constraint for both arms
    bool ac_params_changed;
    custom_msgs::ActiveConstraintParameters ac_parameters;

    KDL::Frame tool_desired_pose_kdl[2];

    uint destination_ring_counter;
    vtkSmartPointer<vtkMatrix4x4> tool_current_pose[2];

//    std::vector<vtkSmartPointer<vtkProp>>           graphics_actors;

    // graphics_actors that are updated during the task
    vtkSmartPointer<vtkActor>                       ring_actor[2];

    vtkSmartPointer<vtkAxesActor>                   tool_current_frame_axes[2];
    vtkSmartPointer<vtkAxesActor>                   tool_desired_frame_axes[2];

    vtkSmartPointer<vtkCellLocator>                 cellLocator;

    vtkSmartPointer<vtkActor>                       mesh_actor;
//    vtkSmartPointer<vtkActor>                       ring_guides_mesh_actor;
//    vtkSmartPointer<vtkActor>                       destination_cone_actor;
//    vtkSmartPointer<vtkActor>                       kidney_mesh_actor ;
    vtkSmartPointer<vtkCornerAnnotation>            cornerAnnotation;

};

#endif //TELEOP_VISION_TaskKidney_H
