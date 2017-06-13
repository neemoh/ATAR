//
// Created by nima on 13/06/17.
//

#include "BulletTask.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>

BulletTask::BulletTask(const std::string stl_file_dir,
                 const bool show_ref_frames, const bool biman,
                 const bool with_guidance)
        :
        VTKTask(show_ref_frames, biman, with_guidance),
        stl_files_dir(stl_file_dir)
{



    // -------------------------------------------------------------------------
    //  ACTIVE CONSTRAINT
    // -------------------------------------------------------------------------
    // these parameters could be set as ros parameters too but since
    // they change during the task I am hard coding them here.
    ac_parameters.method = 0; // 0 for visco/elastic
    ac_parameters.active = 0;

    ac_parameters.max_force = 4.0;
    ac_parameters.linear_elastic_coeff = 1000.0;
    ac_parameters.linear_damping_coeff = 10.0;

    ac_parameters.max_torque = 0.03;
    ac_parameters.angular_elastic_coeff = 0.04;
    ac_parameters.angular_damping_coeff = 0.002;

    d_board_actor = vtkSmartPointer<vtkActor>::New();




    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);


//    // -------------------------------------------------------------------------
//    // Create a cube for the floor
//    vtkSmartPointer<vtkCubeSource> floor_source =
//            vtkSmartPointer<vtkCubeSource>::New();
//    double floor_dimensions[3] = {0.1, 0.09, 0.001};
//    floor_source->SetXLength(floor_dimensions[0]);
//    floor_source->SetYLength(floor_dimensions[1]);
//    floor_source->SetZLength(floor_dimensions[2]);
//    vtkSmartPointer<vtkPolyDataMapper> floor_mapper =
//            vtkSmartPointer<vtkPolyDataMapper>::New();
//    floor_mapper->SetInputConnection(floor_source->GetOutputPort());
//    vtkSmartPointer<vtkActor> floor_actor = vtkSmartPointer<vtkActor>::New();
//    floor_actor->SetMapper(floor_mapper);
//    floor_actor->SetPosition(floor_dimensions[0] / 2, floor_dimensions[1] / 2,
//                             -floor_dimensions[2]);
//    floor_actor->GetProperty()->SetOpacity(0.3);
//    double DeepPink[3] {1.0, 0.08, 0.58};
//    floor_actor->GetProperty()->SetColor(DeepPink);

    // -------------------------------------------------------------------------
    // Create a cube for the board
    vtkSmartPointer<vtkCubeSource> board_source =
            vtkSmartPointer<vtkCubeSource>::New();
    board_dimensions[0]  = 0.18;
    board_dimensions[1]  = 0.14;
    board_dimensions[2]  = 0.5;

    board_source->SetXLength(board_dimensions[0]);
    board_source->SetYLength(board_dimensions[1]);
    board_source->SetZLength(board_dimensions[2]);
    vtkSmartPointer<vtkPolyDataMapper> board_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    board_mapper->SetInputConnection(board_source->GetOutputPort());
    d_board_actor->SetMapper(board_mapper);
    d_board_actor->SetPosition(board_dimensions[0] / 2,
                               board_dimensions[1] / 2,
                               -board_dimensions[2]);
    d_board_actor->GetProperty()->SetOpacity(1.00);
    double colr[3] {1.0, 1.0, 1.0};
    d_board_actor->GetProperty()->SetColor(colr);




    // -------------------------------------------------------------------------
    // Error history spheres

    vtkSmartPointer<vtkSphereSource>  source =
            vtkSmartPointer<vtkSphereSource>::New();

    source->SetRadius(RAD_SPHERES);
    source->SetPhiResolution(30);
    source->SetThetaResolution(30);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(source->GetOutputPort());

    vtkSmartPointer<vtkMinimalStandardRandomSequence> sequence =
            vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
    // initialize the sequence
    sequence->SetSeed(1);

    for (int i = 0; i < NUM_SPHERES; ++i) {

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        sequence->GetRangeValue(0.0,1.0);
        double a = sequence->GetRangeValue(0.0,1.0);
        actor->GetProperty()->SetColor(0.6 - 0.2*a, 0.6 - 0.3*a, 0.7 + 0.3*a);
        sequence->Next();

        a = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double b = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double c = sequence->GetRangeValue(0.0, 0.05);
        sequence->Next();
        sphere_positions.push_back({a,b, 0.07 + c});
        actor->SetPosition(sphere_positions[i][0],
                           sphere_positions[i][1],
                           sphere_positions[i][2]);
        actor->GetProperty()->SetSpecular(0.8);
        actor->GetProperty()->SetSpecularPower(50);
        d_sphere_actors.push_back(actor);
    }


    // -------------------------------------------------------------------------
    double source_scales = 0.006;


    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);
    }
    for (int j = 0; j < d_sphere_actors.size(); ++j) {
        actors.push_back(d_sphere_actors[j]);
    }

    actors.push_back(d_board_actor);



    dInitODE ();

    InitODE();

}


//------------------------------------------------------------------------------
void BulletTask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                        const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

//------------------------------------------------------------------------------
void BulletTask::UpdateActors() {

    for (int i = 0; i < 5; ++i) {
        SimLoopODE();
    }


}




//------------------------------------------------------------------------------
bool BulletTask::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters BulletTask::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState BulletTask::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void BulletTask::ResetTask() {
    ROS_INFO("Resetting the task.");
//    number_of_repetition = 0;
//    task_state = BulletTaskState::RepetitionComplete;
//    ResetOnGoingEvaluation();
//    ResetScoreHistory();
}

void BulletTask::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
//    if(task_state== BulletTaskState::ToEndPoint ||
//       task_state == BulletTaskState::ToStartPoint){
//
//        ResetOnGoingEvaluation();
//        if(number_of_repetition>0)
//            number_of_repetition--;
//        task_state = BulletTaskState::RepetitionComplete;
//
//    }
}


void BulletTask::FindAndPublishDesiredToolPose() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            ("/PSM1/tool_pose_desired", 10);
    if(bimanual)
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

//        CalculatedDesiredToolPose();

        // publish desired poses
        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/task_space";
            pose_msg.header.stamp = ros::Time::now();
            // publish
            pub_desired[n_arm].publish(pose_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}





void BulletTask::InitODE() {


}

void BulletTask::CloseODE() {

}

void BulletTask::SimLoopODE() {

}


void nearCallback(void *data, dGeomID o1, dGeomID o2) {


}



void BulletTask::DrawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb,
                       size_t obj_idx) {


}

BulletTask::~BulletTask() {
//    CloseODE();
}


