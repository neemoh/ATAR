//
// Created by nima on 21/07/17.
//

#include "TaskRingTransfer.h"
#include <custom_conversions/Conversions.h>
#include <boost/thread/thread.hpp>


TaskRingTransfer::TaskRingTransfer(ros::NodeHandlePtr n)
        :
        SimTask(n, 100),
        time_last(ros::Time::now()) {

    InitBullet();

    SimObject *board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    {
        board_dimensions[0] = 0.14;
        board_dimensions[1] = 0.12;
        board_dimensions[2] = 0.01;

        double friction = 0.5;

        KDL::Frame pose;
        pose.p = KDL::Vector(
                - 0.01 + board_dimensions[0]/2,  board_dimensions[1]/2,
                -board_dimensions[2]/2);

        std::vector<double> dim = {
                board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose,
                              0.0, friction);
        //    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.6, 0.5, 0.5);

        dynamics_world->addRigidBody(board->GetBody());
        graphics_actors.push_back(board->GetActor());
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject* floor= new SimObject(ObjectShape::STATICPLANE,
                                    ObjectType::DYNAMIC, floor_dims);
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create destination rods
    {
        int cols = 4;
        int rows = 1;
        float friction = 2.2;
        SimObject *cylinders[cols * rows];
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {

                std::vector<double> dim = {0.002, 0.035};

                KDL::Frame pose(KDL::Rotation::Quaternion(0, 0.70711,
                                                          0.70711, 0.0),
                                KDL::Vector( 0.02 + (double) j * 0.028, 0.09,
                                             dim[1] / 2));

                cylinders[i * rows + j] =
                        new SimObject(ObjectShape::CYLINDER,
                                      ObjectType::DYNAMIC, dim, pose, 0.0,
                                      friction);

                auto ratio = (float) j / (float) cols;
                cylinders[i * rows + j]->GetActor()->GetProperty()->SetColor(
                        0.9 - 0.3 * ratio, 0.4, 0.4 + 0.4 * ratio);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecular(0.8);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecularPower(50);

                dynamics_world->addRigidBody(cylinders[i * rows + j]->GetBody());
                graphics_actors.push_back(cylinders[i * rows + j]->GetActor());

            }
        }
    }

    // -------------------------------------------------------------------------
    // ROD
    {
        std::vector<double> rod_dim = {0.002, 0.1};
        KDL::Frame pose(KDL::Rotation::Quaternion(0.70711, 0.70711, 0.0, 0.0),
                        KDL::Vector(0.10, 0.03, 0.03));
        SimObject rod = SimObject(ObjectShape::CYLINDER, ObjectType::DYNAMIC,
                                  rod_dim, pose);

        dynamics_world->addRigidBody(rod.GetBody());
        graphics_actors.push_back(rod.GetActor());
        rod.GetActor()->GetProperty()->SetColor(0.3, 0.3, 0.3);

    }
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    //// Create smallRING meshes
    {
        size_t n_rings = 4;
        SimObject *rings[n_rings];
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_Hook_ring_D2cm_D5mm.obj");
        std::string mesh_file_dir_str = input_file_dir.str();
        float density = 50000;
        float friction = 5;
        for (int l = 0; l < n_rings; ++l) {

            KDL::Frame pose(KDL::Rotation::Quaternion(0.70711, 0.70711, 0.0, 0.0),
                            KDL::Vector(0.06 + (double) l * 0.01, 0.03, 0.03));
            std::vector<double> dim; // not used

            rings[l] = new
                    SimObject(ObjectShape::MESH, ObjectType::DYNAMIC, dim, pose,
                              density, friction,
                              mesh_file_dir_str, 0);
            dynamics_world->addRigidBody(rings[l]->GetBody());
            graphics_actors.push_back(rings[l]->GetActor());
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

        grippers[0] = new FiveLinkGripper(gripper_link_dims);

        for (int j = 0; j < 1 ;++j) {
            grippers[j]->AddToWorld(dynamics_world);
            grippers[j]->AddToActorsVector(graphics_actors);
        }

        std::vector<std::vector<double>> gripper_3link_dims =
                {{0.002, 0.002, 0.005}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.009}};

    }


    // -------------------------------------------------------------------------
    //// Create hook mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.09, 0.07, 0.08));
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_hook_hook.obj");
        std::string mesh_file_dir_str = input_file_dir.str();
        std::vector<double> dim; // not used
        float density = 50000;

        hook_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::KINEMATIC, dim, pose,
                          density, 0,
                          mesh_file_dir_str, 0);
        dynamics_world->addRigidBody(hook_mesh->GetBody());
        graphics_actors.push_back(hook_mesh->GetActor());
        hook_mesh->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);
    }

    // -------------------------------------------------------------------------
    //// Create rod tool
    //{
    //    std::vector<double> dim = {0.0015, 0.04};
    //    double pose[7]{0.0, 0.0, 0.0, 0.7, 0, 0.7, 0};
    //
    //    tool_cyl = new SimObject(
    //        ObjectShape::CYLINDER, ObjectType::KINEMATIC, dim, pose,
    //       );
    //    dynamics_world->addRigidBody(tool_cyl->GetBody());
    //    graphics_actors.push_back(tool_cyl->GetActor());
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
    bool show_ref_frames = 0;
    if(show_ref_frames)
        graphics_actors.push_back(task_coordinate_axes);


    graphics = new Rendering(n);

    // Define a master manipulator
    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");
    //    master = new Manipulator(nh, "/dvrk/MTML",
    //                                   "/position_cartesian_current",
    //                                   "/gripper_position_current",
    //                                   cam_pose);
    graphics = new Rendering(n);

    graphics->AddActorsToScene(GetActors());
};

//------------------------------------------------------------------------------
void TaskRingTransfer::StepWorld() {

    graphics->Render();

    //-------------------------------- UPDATE RIGHT GRIPPER
    KDL::Frame grpr_right_pose;
    master->GetPoseWorld(grpr_right_pose);
    // map gripper value to an angle

    double grip_posit;
    master->GetGripper(grip_posit);
    double theta_min=0*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    grippers[0]->SetPoseAndJawAngle(grpr_right_pose, grip_angle);

    //Update the pose of hook
    {
        KDL::Frame tool_pose;
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
    StepPhysics();

}


custom_msgs::TaskState TaskRingTransfer::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskRingTransfer::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskRingTransfer::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskRingTransfer::HapticsThread() {

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

//        CalculatedDesiredToolPose();

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





void TaskRingTransfer::InitBullet() {

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


void TaskRingTransfer::StepPhysics() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();
    // check http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();
}


TaskRingTransfer::~TaskRingTransfer() {

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

    delete master;

    delete graphics;



}


