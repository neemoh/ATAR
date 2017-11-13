//
// Created by nima on 13/06/17.
//

#include "TaskNeedle.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <math.h>



TaskNeedle::TaskNeedle(ros::NodeHandlePtr n)
        :
        SimTask(n),
        time_last(ros::Time::now()) {

    bool ar_mode = true;
    int n_views = 3;
    bool one_window_per_view = false;
    bool borders_off  = true;
    std::vector<int> view_resolution = {640, 480};
    std::vector<int> window_positions={1280, 0};

    graphics = std::make_unique<Rendering>(n, view_resolution, ar_mode, n_views,
                                           one_window_per_view, borders_off,window_positions);

    // Define a master manipulator
    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");
    graphics->SetManipulatorInterestedInCamPose(master);

    // -------------------------------------------------------------------------
    // Create a cube for the board
    board_dimensions[0] = 0.14;
    board_dimensions[1] = 0.12;
    board_dimensions[2] = 0.01;
    {
        double friction = 0.5;

        KDL::Frame pose(KDL::Rotation::Quaternion(0., 0, 0.0, 1),
                        KDL::Vector(board_dimensions[0] / 2.45,
                                    board_dimensions[1] / 2.78,
                                    -board_dimensions[2] / 2));
        std::vector<double> dim = {
                board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose,
                              0.0, friction);
        //    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.5, 0.3, 0.1);
        AddSimObjectToTask(board);
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE, floor_dims);
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    //// Create needle mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.04, 0.11, 0.03));
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 90000; // kg/m3
        needle_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                          RESOURCES_DIRECTORY
                          +"/mesh/task_needle_needle_L3cm_d3mm.obj", pose,
                          density, friction);
        AddSimObjectToTask(needle_mesh);
        needle_mesh->GetActor()->GetProperty()->SetColor(0.8f, 0.8f, 0.8f);
    }


    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    SimObject *suture_plane_1;
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.5, 0.5, 0.5, 0.5),
                        KDL::Vector(0.055, 0.07, 0.02));
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        suture_plane_1 = new SimObject(ObjectShape::MESH, ObjectType::DYNAMIC
                ,RESOURCES_DIRECTORY + "/mesh/task_needle_suture_plane.obj",
                                       pose, density, friction);


        AddSimObjectToTask(suture_plane_1);
        suture_plane_1->GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
    }

    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    SimObject * suture_plane_2;
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.5, 0.5, 0.5, 0.5),
                        KDL::Vector(0.08, 0.07, 0.02));
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        suture_plane_2 = new SimObject(ObjectShape::MESH, ObjectType::DYNAMIC
                ,RESOURCES_DIRECTORY + "/mesh/task_needle_suture_plane.obj"
                , pose, density, friction);
        suture_plane_2->GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
        AddSimObjectToTask(suture_plane_2);
    }


    // -------------------------------------------------------------------------
    //// Create ring mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.07, 0.02, 0.08));
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 50000; // kg/m3
        ring_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim, pose,
                          density, friction,
                RESOURCES_DIRECTORY + "/mesh/task_needle_ring_D2cm_D5mm.obj" );
        AddSimObjectToTask(ring_mesh);
        ring_mesh->GetActor()->GetProperty()->SetColor(0.4f, 0.3f, 0.3f);
        //ring_mesh->GetBody()->setContactStiffnessAndDamping(5000, 10);
    }

    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    bool show_ref_frames = 1;
    if(show_ref_frames)
        graphics->AddActorToScene(task_coordinate_axes);



};

//------------------------------------------------------------------------------
void TaskNeedle::TaskLoop() {

    KDL::Frame grpr_right_pose;
    master->GetPoseWorld(grpr_right_pose);
    double grip_posit;
    master->GetGripper(grip_posit);


    //--------------------------------
    // step the world
    StepPhysics();

}


custom_msgs::TaskState TaskNeedle::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskNeedle::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskNeedle::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskNeedle::HapticsThread() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            ("/PSM1/tool_pose_desired", 10);
//    if(bimanual)
//        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
//                ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

////        CalculatedDesiredToolPose();
//
//        // publish desired poses
//        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {
//
//            // convert to pose message
//            geometry_msgs::PoseStamped pose_msg;
//            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
//            // fill the header
//            pose_msg.header.frame_id = "/task_space";
//            pose_msg.header.stamp = ros::Time::now();
//            // publish
//            pub_desired[n_arm].publish(pose_msg);
//        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}


void TaskNeedle::StepPhysics() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();
}


TaskNeedle::~TaskNeedle() {

    delete master;

}

void TaskNeedle::UpdateGripperLinksPose(const KDL::Frame pose,
                                        const double grip_angle,
                                        const std::vector<std::vector<double> > gripper_link_dims,
                                        SimObject *link_objects[]
) {
    KDL::Frame grpr_links_pose[5];

    //-------------------------------- LINK 0
    grpr_links_pose[0] = pose;
    grpr_links_pose[0].p  = grpr_links_pose[0] * KDL::Vector( 0.0 , 0.0,
                                                              -gripper_link_dims[0][2]/2);
    double x, y, z, w;
    pose.M.GetQuaternion(x,y,z,w);
    double link0_pose[7] = {grpr_links_pose[0].p.x(),
                            grpr_links_pose[0].p.y(), grpr_links_pose[0].p.z(),x,y,z,w};
    link_objects[0]->SetKinematicPose(link0_pose);

    //-------------------------------- LINK 1
    grpr_links_pose[1] = pose;
    grpr_links_pose[1].M.DoRotX(-grip_angle);
    grpr_links_pose[1].p =  grpr_links_pose[1] *
                            KDL::Vector( 0.0, 0.0, gripper_link_dims[1][2]/2);
    grpr_links_pose[1].M.GetQuaternion(x, y, z, w);

    double link2_pose[7] = {grpr_links_pose[1].p.x(),
                            grpr_links_pose[1].p.y(), grpr_links_pose[1].p.z(), x, y, z, w};

    link_objects[1]->SetKinematicPose(link2_pose);

    //-------------------------------- LINK 2
    grpr_links_pose[2] = pose;
    grpr_links_pose[2].M.DoRotX(grip_angle);
    grpr_links_pose[2].p =  grpr_links_pose[2] *
                            KDL::Vector( 0.0, 0.0, gripper_link_dims[2][2]/2);
    grpr_links_pose[2].M.GetQuaternion(x, y, z, w);

    double link3_pose[7] = {grpr_links_pose[2].p.x(),
                            grpr_links_pose[2].p.y(), grpr_links_pose[2].p.z(), x, y, z, w};

    link_objects[2]->SetKinematicPose(link3_pose);


    //-------------------------------- LINKS 3 and $
    for (int i = 3; i < 5; ++i) {
        // first find the end point of links 1 and 2 and then add half length
        // of links 3 and 4
        grpr_links_pose[i] = pose;
        grpr_links_pose[i].p =
                grpr_links_pose[i-2] *
                KDL::Vector(0., 0.,gripper_link_dims[i-2][2]/2)
                + grpr_links_pose[i].M *
                  KDL::Vector(0., 0.,gripper_link_dims[i][2]/2);

        grpr_links_pose[i].M.GetQuaternion(x,y,z,w);
        double link_pose[7] = {grpr_links_pose[i].p.x(),
                               grpr_links_pose[i].p.y(), grpr_links_pose[i].p.z(),x, y, z, w};

        link_objects[i]->SetKinematicPose(link_pose);
    }

}


