//
// Created by nima on 21/07/17.
//

#include "TaskHook.h"

#include <custom_conversions/Conversions.h>
#include <boost/thread/thread.hpp>


TaskHook::TaskHook(const std::string mesh_files_dir,
                   const bool show_ref_frames, const bool biman,
                   const bool with_guidance)
        :
        VTKTask(show_ref_frames, biman, with_guidance, 0),
        time_last(ros::Time::now()) {

    InitBullet();

    BulletVTKObject *board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    {
        board_dimensions[0] = 0.14;
        board_dimensions[1] = 0.12;
        board_dimensions[2] = 0.01;

        double friction = 0.5;

        double pose[7]{
                - 0.01 + board_dimensions[0]/2,  board_dimensions[1]/2,
                -board_dimensions[2]/2, 0, 0, 0, 1};

        std::vector<double> dim = {
                board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new BulletVTKObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim,
                                    pose, 0.0, 0, friction,
                                    NULL);
        //    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.6, 0.5, 0.5);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject* floor= new BulletVTKObject(ObjectShape::STATICPLANE,
                                                ObjectType::DYNAMIC, floor_dims,
                                                dummy_pose, 0.0, 0, 0,
                                                NULL);
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create destination rods
    {
        int cols = 4;
        int rows = 1;
        float friction = 2.2;
        BulletVTKObject *cylinders[cols * rows];
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {

                std::vector<double> dim = {0.002, 0.035};

                double pose[7] = {
                        0.02 + (double) j * 0.028, 0.09, dim[1] / 2, 0, 0.70711
                        , 0.70711, 0.0
                };

                cylinders[i * rows + j] =
                        new BulletVTKObject(ObjectShape::CYLINDER,
                                            ObjectType::DYNAMIC, dim, pose, 0.0,
                                            0, friction,
                                            NULL);

                auto ratio = (float) j / (float) cols;
                cylinders[i * rows + j]->GetActor()->GetProperty()->SetColor(
                        0.9 - 0.3 * ratio, 0.4, 0.4 + 0.4 * ratio);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecular(0.8);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecularPower(50);

                dynamics_world->addRigidBody(cylinders[i * rows + j]->GetBody());
                actors.push_back(cylinders[i * rows + j]->GetActor());

            }
        }
    }

    // -------------------------------------------------------------------------
    // ROD
    {
        std::vector<double> rod_dim = {0.002, 0.1};
        double pose[7] = {0.10, 0.03, 0.03, 0.70711, 0.70711, 0.0, 0.0};

        BulletVTKObject rod = BulletVTKObject(ObjectShape::CYLINDER,
                                              ObjectType::DYNAMIC, rod_dim,
                                              pose, 0.0, 0, 0,
                                              NULL);

        dynamics_world->addRigidBody(rod.GetBody());
        actors.push_back(rod.GetActor());
        rod.GetActor()->GetProperty()->SetColor(0.3, 0.3, 0.3);

    }
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    //// Create smallRING meshes
    {
        size_t n_rings = 4;
        BulletVTKObject *rings[n_rings];
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("ring_D2cm_D5mm.obj");
        std::string mesh_file_dir_str = input_file_dir.str();
        float density = 50000;
        float friction = 5;
        for (int l = 0; l < n_rings; ++l) {

            double pose[7]{
                    0.06 + (double) l * 0.01, 0.03, 0.03, 0.70711, 0.70711, 0.0, 0.0
            };

            std::vector<double> dim; // not used

            rings[l] = new
                    BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC, dim,
                                    pose, density, 0, friction,
                                    &mesh_file_dir_str);
            dynamics_world->addRigidBody(rings[l]->GetBody());
            actors.push_back(rings[l]->GetActor());
            rings[l]->GetActor()->GetProperty()->SetColor(0., 0.5, 0.6);

        }
    }
// -------------------------------------------------------------------------
    // Create kinematic gripper
    //
    // The gripper is a 5 link mechanism, link 0 is a base link, 1st and 2nd
    // are angular jaws (like a scissor) so their orientation is related to
    // the gripper angle of the master, 3rd and 4th change position along one
    // axis according to the gripper angle, like a clamp. These last links
    // are used to generate enough normal force for a stable grasping, since
    // the scissor type links push the objects outwards.
    // make sure tu set a high friction coefficient for the objects you want
    // to grasp.
    // In addition, make sure you limit the gripper angle so that the user
    // can't press the object too much. Otherwise the injected energy would
    // be so high that no friction can compensate it.
    {

        double gripper_pose[7]{0, 0, 0, 0, 0, 0, 1};
        gripper_link_dims =
                {{0.003, 0.003, 0.005}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.007}
                        , {0.004, 0.001, 0.007}};

        grippers[0] = new SimpleGripper(gripper_link_dims);

        for (int j = 0; j < 1 ;++j) {
            grippers[j]->AddToWorld(dynamics_world);
            grippers[j]->AddToActorsVector(actors);
        }


        std::vector<std::vector<double>> gripper_3link_dims =
                {{0.002, 0.002, 0.005}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.009}};

        three_gripper = new ThreeLinkGripper(gripper_3link_dims);

        three_gripper->AddToWorld(dynamics_world);
        three_gripper->AddToActorsVector(actors);

        //for (int i = 0; i < 5; ++i) {
        //    left_gripper_links[i] =
        //        new BulletVTKObject(
        //            ObjectShape::BOX, ObjectType::KINEMATIC,
        //            gripper_link_dims[i], gripper_pose,
        //            gripper_density,
        //            NULL, gripper_friction
        //        );
        //    dynamics_world->addRigidBody(left_gripper_links[i]->GetBody());
        //    actors.push_back(left_gripper_links[i]->GetActor());
        //    left_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
        //    left_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(50);
        //    left_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);
        //}

    }


    // -------------------------------------------------------------------------
    //// Create hook mesh
    {
        double pose[7]{0.09, 0.07, 0.08, 0.7, 0, 0.7, 0};
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("hook.obj");
        std::string mesh_file_dir_str = input_file_dir.str();
        std::vector<double> dim; // not used
        float density = 50000;

        hook_mesh = new
                BulletVTKObject(ObjectShape::MESH, ObjectType::KINEMATIC, dim,
                                pose, density, 0, 0,
                                &mesh_file_dir_str);
        dynamics_world->addRigidBody(hook_mesh->GetBody());
        actors.push_back(hook_mesh->GetActor());
        hook_mesh->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);
    }

    // -------------------------------------------------------------------------
    //// Create rod tool
    //{
    //    std::vector<double> dim = {0.0015, 0.04};
    //    double pose[7]{0.0, 0.0, 0.0, 0.7, 0, 0.7, 0};
    //
    //    tool_cyl = new BulletVTKObject(
    //        ObjectShape::CYLINDER, ObjectType::KINEMATIC, dim, pose,
    //        0.0, NULL);
    //    dynamics_world->addRigidBody(tool_cyl->GetBody());
    //    actors.push_back(tool_cyl->GetActor());
    //    tool_cyl->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);
    //}

    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    if(show_ref_frames)
        actors.push_back(task_coordinate_axes);

}


//------------------------------------------------------------------------------
void TaskHook::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                         const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void TaskHook::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    jaw_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskHook::UpdateActors() {


    //-------------------------------- UPDATE RIGHT GRIPPER
    KDL::Frame grpr_right_pose = (*tool_current_pose_kdl[0]);
    // map gripper value to an angle
    double grip_posit = (*jaw_position[0]);
    double theta_min=0*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    //grippers[0]->SetPoseAndJawAngle(grpr_right_pose, grip_angle);

    double y = 0.05 * sin(M_PI*(double)counter/50.);
    counter++;
    double grip_angle_1 = theta_min + theta_min * sin(M_PI*(double)counter/40.);
    KDL::Frame th_gripper_pose;
    th_gripper_pose.p = KDL::Vector(0.03 + y , 0.03 , 0.01);
    th_gripper_pose.M.DoRotY(M_PI/2);
    th_gripper_pose.M.DoRotZ(M_PI/2);
    three_gripper->SetPoseAndJawAngle(grpr_right_pose, grip_angle);


    //Update the pose of hook
    {
        KDL::Frame tool_pose = (*tool_current_pose_kdl[1]);
        KDL::Frame local_tool_transform;

        // locally transform the tool if needed
        local_tool_transform.p = KDL::Vector(0.0, 0.0, 0);
        local_tool_transform.M.DoRotY(0);
        tool_pose = tool_pose * local_tool_transform;

        //KDL::Vector box_posit = tool_pose * KDL::Vector(0.0, 0.0, 0);

        double x, y, z, w;
        tool_pose.M.GetQuaternion(x, y, z, w);
        double box_pose[7] = {tool_pose.p.x(), tool_pose.p.y(), tool_pose.p.z(),
                              x, y, z, w};
        hook_mesh->SetKinematicPose(box_pose);

    }



    //--------------------------------
    // step the world
    StepDynamicsWorld();

}




//------------------------------------------------------------------------------
bool TaskHook::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters * TaskHook::GetACParameters() {
    custom_msgs::ActiveConstraintParameters* msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState TaskHook::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskHook::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskHook::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskHook::FindAndPublishDesiredToolPose() {

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





void TaskHook::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamics_world = new btDiscreteDynamicsWorld(dispatcher,
                                                 overlappingPairCache, solver,
                                                 collisionConfiguration);

    dynamics_world->setGravity(btVector3(0, 0, -10));

    btContactSolverInfo& info = dynamics_world->getSolverInfo();
    //optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
    //only enable split impulse position correction when the penetration is
    // deeper than this m_splitImpulsePenetrationThreshold, otherwise use the
    // regular velocity/position constraint coupling (Baumgarte).
    info.m_splitImpulsePenetrationThreshold = -0.02f;
    info.m_numIterations = 15;
    info.m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS;

}


void TaskHook::StepDynamicsWorld() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();
    // check http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();
}


TaskHook::~TaskHook() {

    ROS_INFO("Destructing Bullet task: %d",
             dynamics_world->getNumCollisionObjects());
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamics_world->removeCollisionObject(obj);
        delete obj;
    }

    //delete dynamics world
    delete dynamics_world;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;


}


