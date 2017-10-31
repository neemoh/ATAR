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
        SimTask(n, 100),
        time_last(ros::Time::now()) {

    InitBullet();


    // -----------------------
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

        dynamics_world->addRigidBody(board->GetBody());
        graphics_actors.push_back(board->GetActor());
        //        board->GetBody()->
        //                setCollisionFlags(board->GetBody()->getCollisionFlags() |
        //                                  btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

        board->GetBody()->setUserPointer(board);
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE,
                                     ObjectType::DYNAMIC, floor_dims,
                                     KDL::Frame(), 0.0);
    dynamics_world->addRigidBody(floor->GetBody());

     // -------------------------------------------------------------------------
    //// Create needle mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.04, 0.11, 0.03));
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 90000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_needle_needle_L3cm_d3mm"
                                                                ".obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        needle_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim, pose,
                          density, friction,
                          mesh_file_dir_str, 1);

        dynamics_world->addRigidBody(needle_mesh->GetBody());
        graphics_actors.push_back(needle_mesh->GetActor());
        needle_mesh->GetActor()->GetProperty()->SetColor(0.8f, 0.8f, 0.8f);

//        needle_mesh->GetBody()->
//                setCollisionFlags(needle_mesh->GetBody()->getCollisionFlags() |
//                                  btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

        needle_mesh->GetBody()->setUserPointer(needle_mesh);
    }


    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.5, 0.5, 0.5, 0.5),
                        KDL::Vector(0.055, 0.07, 0.02));
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_needle_suture_plane.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        SimObject suture_plane_1(ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
                                 pose, density, friction,
                                 mesh_file_dir_str, 0);

        dynamics_world->addRigidBody(suture_plane_1.GetBody());
        graphics_actors.push_back(suture_plane_1.GetActor());
        suture_plane_1.GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
    }

    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.5, 0.5, 0.5, 0.5),
                        KDL::Vector(0.08, 0.07, 0.02));
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_needle_suture_plane.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        SimObject suture_plane_2(ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
                                 pose, density, friction,
                                 mesh_file_dir_str, 0);

        dynamics_world->addRigidBody(suture_plane_2.GetBody());
        graphics_actors.push_back(suture_plane_2.GetActor());
        suture_plane_2.GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
    }


    // -------------------------------------------------------------------------
    //// Create ring mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.07, 0.02, 0.08));
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 50000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << MESH_DIRECTORY << std::string("task_needle_ring_D2cm_D5mm.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        ring_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim, pose,
                          density, friction,
                          mesh_file_dir_str, 0);

        dynamics_world->addRigidBody(ring_mesh->GetBody());
        graphics_actors.push_back(ring_mesh->GetActor());
        ring_mesh->GetActor()->GetProperty()->SetColor(0.4f, 0.3f, 0.3f);
        //ring_mesh->GetBody()->setContactStiffnessAndDamping(5000, 10);
    }

    // -------------------------------------------------------------------------
    // Create kinematic jaw (gripper)
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
        float gripper_density = 0; // kg/m3
        float gripper_friction = 10;
        gripper_link_dims =
                {{0.003, 0.003, 0.005},
                 {0.004, 0.001, 0.009},
                 {0.004, 0.001, 0.009},
                 {0.004, 0.001, 0.007},
                 {0.004, 0.001, 0.007}};


        for (int i = 0; i < 5; ++i) {
            right_gripper_links[i] =
                    new SimObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                  gripper_link_dims[i], KDL::Frame(),
                                  gripper_density, gripper_friction);
            dynamics_world->addRigidBody(right_gripper_links[i]->GetBody());
            graphics_actors.push_back(right_gripper_links[i]->GetActor());
            right_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,
                                                                        0.7f,
                                                                        0.7f);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(
                    50);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);
        }


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
        graphics_actors.push_back(task_coordinate_axes);

    graphics = new Rendering(nh, false, 3);

    // Define a master manipulator
    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");
    //    master = new Manipulator(nh, "/dvrk/MTML",
    //                                   "/position_cartesian_current",
    //                                   "/gripper_position_current",
    //                                   cam_pose);

    graphics->AddActorsToScene(GetActors());
};

//------------------------------------------------------------------------------
void TaskNeedle::StepWorld() {

    graphics->Render();
//    MyContactResultCallback result;
//    dynamics_world->contactPairTest(needle_mesh->GetBody(),
//                                    board->GetBody(),
//                                    result);
//    std::cout << "in " << result.connected << std::endl;
    KDL::Frame grpr_right_pose;
    master->GetPoseWorld(grpr_right_pose);
    double grip_posit;
    master->GetGripper(grip_posit);
    //-------------------------------- UPDATE RIGHT GRIPPER
    // map gripper value to an angle
    double theta_min=14*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    UpdateGripperLinksPose(grpr_right_pose, grip_angle, gripper_link_dims,
                           right_gripper_links);


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





void TaskNeedle::InitBullet() {

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


void TaskNeedle::StepPhysics() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();
}


TaskNeedle::~TaskNeedle() {

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


