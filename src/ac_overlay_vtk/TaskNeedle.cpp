//
// Created by nima on 13/06/17.
//

#include "TaskNeedle.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#define _USE_MATH_DEFINES


bool ContactAddedCallbackBullet(btManifoldPoint& cp,const
btCollisionObjectWrapper* obj1,int id1,int index1,const btCollisionObjectWrapper* obj2,int id2,int index2)
{
    auto obj_p1 = (BulletVTKObject*)obj1->getCollisionObject()
            ->getUserPointer();
    auto obj_p2 = (BulletVTKObject*)obj2->getCollisionObject()
            ->getUserPointer();

    if(obj_p1 && obj_p2) {
        std::cout << "collision!" << obj_p1->GetId() << "  " << obj_p2->GetId()
                  << "\n";
        obj_p1->GetActor()->GetProperty()->SetColor(1., 0.0, 0.0);
        obj_p2->GetActor()->GetProperty()->SetColor(1., 0.0, 0.0);
    }
    return false;
}
ContactAddedCallback gContactAddedCallback = ContactAddedCallbackBullet;

TaskNeedle::TaskNeedle(const std::string mesh_files_dir,
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

    //board_dimensions[0]  = 0.18;
    //board_dimensions[1]  = 0.14;
    //board_dimensions[2]  = 0.1;

    board_dimensions[0] = 0.14;
    board_dimensions[1] = 0.12;
    board_dimensions[2] = 0.01;
    //double stiffnes = 1000;
    //double damping = 20;
    {
        double friction = 0.5;

        double pose[7]{
            board_dimensions[0] / 2.45, board_dimensions[1] / 2.78,
            -board_dimensions[2] / 2, 0, 0, 0, 1
        };

        std::vector<double> dim = {
            board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new BulletVTKObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim,
                                    pose, 0.0, 0, friction,
                                    NULL);
//    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.5, 0.3, 0.1);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
        board->GetBody()->
                setCollisionFlags(board->GetBody()->getCollisionFlags() |
                                  btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

        board->GetBody()->setUserPointer(board);
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject *floor = new BulletVTKObject(ObjectShape::STATICPLANE,
                                                 ObjectType::DYNAMIC,
                                                 floor_dims, dummy_pose, 0.0, 0,
                                                 0,
                                                 NULL);
    dynamics_world->addRigidBody(floor->GetBody());

    //// -------------------------------------------------------------------------
    //// Create cylinders
    //{
    //    uint cols = 4;
    //    uint rows = 3;
    //    float density = 50000; // kg/m3
    //    float friction = 2.2;
    //    BulletVTKObject *cylinders[cols * rows];
    //    for (int i = 0; i < rows; ++i) {
    //
    //        for (int j = 0; j < cols; ++j) {
    //
    //            std::vector<double> dim = {0.003, 0.03};
    //
    //            double q3 = 0;
    //            double q4 = 1;
    //            if (j < cols / 2) {
    //                q3 = 0.70711;
    //                q4 = 0.70711;
    //            }
    //            double pose[7]{
    //                (double) i * 4 * dim[0] + (double) j * dim[0] / 2, 0.06, 0.1
    //                    + dim[1] * 1.5 * (double) j, 0, 0, q3, q4
    //            };
    //
    //            cylinders[i * rows + j] =
    //                new BulletVTKObject(
    //                    ObjectShape::CYLINDER, ObjectType::DYNAMIC, dim, pose,
    //                    density, NULL, friction
    //                );
    //            double ratio = (double) i / 4.0;
    //            cylinders[i * rows + j]->GetActor()->GetProperty()->SetColor(
    //                0.6 - 0.2 * ratio, 0.6 - 0.3 * ratio, 0.7 + 0.3 * ratio
    //            );
    //            cylinders[i * rows + j]->GetActor()->GetProperty()->SetSpecular(
    //                0.8
    //            );
    //            cylinders[i * rows
    //                + j]->GetActor()->GetProperty()->SetSpecularPower(50);
    //
    //            dynamics_world->addRigidBody(
    //                cylinders[i * rows + j]->GetBody());
    //            actors.push_back(cylinders[i * rows + j]->GetActor());
    //
    //        }
    //    }
    //}
    //// -------------------------------------------------------------------------
    //// Create cubes
    //{
    //    uint rows = 3;
    //    uint cols = 2;
    //    int layers = 3;
    //    BulletVTKObject *cubes[layers * rows * cols];
    //
    //    double sides = 0.006;
    //    float density = 50000; // kg/m3
    //    float friction = 12;
    //
    //    for (int k = 0; k < layers; ++k) {
    //        for (int i = 0; i < rows; ++i) {
    //            for (int j = 0; j < cols; ++j) {
    //
    //                double pose[7]{
    //                    (double) i * 2.2 * sides + 0.1, (double) j * 2.2 * sides
    //                        + 0.05, (double) k * 4 * sides + 0.05, 0, 0, 0, 1
    //                };
    //
    //                std::vector<double> dim = {sides, sides, sides};
    //                cubes[i * rows + j] = new BulletVTKObject(
    //                    ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose,
    //                    density,
    //                    NULL, friction
    //                );
    //                double ratio = (double) i / 4.0;
    //                cubes[i * rows + j]->GetActor()->GetProperty()->SetColor(
    //                    0.6 + 0.1 * ratio, 0.3 - 0.3 * ratio, 0.7 - 0.3 * ratio
    //                );
    //
    //                dynamics_world->addRigidBody(
    //                    cubes[i * rows + j]->GetBody());
    //                actors.push_back(cubes[i * rows + j]->GetActor());
    //
    //            }
    //        }
    //    }
    //}


    // -------------------------------------------------------------------------
    //// Create needle mesh
    {
        double pose[7]{0.04, 0.06, 0.03, 0.7, 0, 0.7, 0};
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 90000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("needle_L3cm_d3mm_mesh"
                                                            ".obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        needle_mesh = new
                BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
                                pose, density, 1, friction,
                                &mesh_file_dir_str);

        dynamics_world->addRigidBody(needle_mesh->GetBody());
        actors.push_back(needle_mesh->GetActor());
        needle_mesh->GetActor()->GetProperty()->SetColor(0.8f, 0.8f, 0.8f);

        needle_mesh->GetBody()->
                setCollisionFlags(needle_mesh->GetBody()->getCollisionFlags() |
        btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

        needle_mesh->GetBody()->setUserPointer(needle_mesh);
    }


    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    {
        double pose[7]{0.055, 0.07, 0.02, 0.5, 0.5, 0.5, 0.5};
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("suture_plane.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        BulletVTKObject suture_plane_1(ObjectShape::MESH, ObjectType::DYNAMIC,
                                       _dim, pose, density, 0, friction,
                                       &mesh_file_dir_str);

        dynamics_world->addRigidBody(suture_plane_1.GetBody());
        actors.push_back(suture_plane_1.GetActor());
        suture_plane_1.GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
    }

    // -------------------------------------------------------------------------
    //// Create suture plane 1 mesh
    {
        double pose[7]{0.08, 0.07, 0.02, 0.5, 0.5, 0.5, 0.5};
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 0; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("suture_plane.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        BulletVTKObject suture_plane_2(ObjectShape::MESH, ObjectType::DYNAMIC,
                                       _dim, pose, density, 0, friction,
                                       &mesh_file_dir_str);

        dynamics_world->addRigidBody(suture_plane_2.GetBody());
        actors.push_back(suture_plane_2.GetActor());
        suture_plane_2.GetActor()->GetProperty()->SetColor(0.8f, 0.2f, 0.2f);
    }


    // -------------------------------------------------------------------------
    //// Create ring mesh
    {
        double pose[7]{0.07, 0.02, 0.08, 0.7, 0, 0.7, 0};
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 50000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("ring_D2cm_d4mm.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        ring_mesh = new
                BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
                                pose, density, 0, friction,
                                &mesh_file_dir_str);

        dynamics_world->addRigidBody(ring_mesh->GetBody());
        actors.push_back(ring_mesh->GetActor());
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
    double gripper_pose[7]{0, 0, 0, 0, 0, 0, 1};
    gripper_link_dims =
        {{0.003, 0.003, 0.005}
         , {0.004, 0.001, 0.009}
         , {0.004, 0.001, 0.009}
         , {0.004, 0.001, 0.007}
         , {0.004, 0.001, 0.007}};


        for (int i = 0; i < 5; ++i) {
            right_gripper_links[i] =
                    new BulletVTKObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                        gripper_link_dims[i], gripper_pose,
                                        gripper_density, 0, gripper_friction,
                                        NULL);
            dynamics_world->addRigidBody(right_gripper_links[i]->GetBody());
            actors.push_back(right_gripper_links[i]->GetActor());
            right_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);
        }

        for (int i = 0; i < 5; ++i) {
            left_gripper_links[i] =
                    new BulletVTKObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                        gripper_link_dims[i], gripper_pose,
                                        gripper_density, 0, gripper_friction,
                                        NULL);
            dynamics_world->addRigidBody(left_gripper_links[i]->GetBody());
            actors.push_back(left_gripper_links[i]->GetActor());
            left_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
            left_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            left_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);
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


    actors.push_back(task_coordinate_axes);


    // set user pointer for collision detection
    for (int j = 0; j < dynamics_world->getNumCollisionObjects() - 1; ++j) {

    }
    }


//------------------------------------------------------------------------------
void TaskNeedle::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void TaskNeedle::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskNeedle::UpdateActors() {

    //-------------------------------- UPDATE RIGHT GRIPPER
    KDL::Frame grpr_right_pose = (*tool_current_pose_kdl[0]);
    // map gripper value to an angle
    double grip_posit = (*gripper_position[0]);
    double theta_min=14*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    UpdateGripperLinksPose(grpr_right_pose, grip_angle, gripper_link_dims,
                           right_gripper_links);

    //-------------------------------- UPDATE LEFT GRIPPER
    KDL::Frame grpr_left_pose = (*tool_current_pose_kdl[1]);
    // map gripper value to an angle
    grip_posit = (*gripper_position[1]);
    grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    UpdateGripperLinksPose(grpr_left_pose, grip_angle, gripper_link_dims,
                           left_gripper_links);

    //--------------------------------
    // step the world
    StepDynamicsWorld();

}


//------------------------------------------------------------------------------
bool TaskNeedle::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters TaskNeedle::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
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


void TaskNeedle::FindAndPublishDesiredToolPose() {

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


void TaskNeedle::StepDynamicsWorld() {
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

//    for (int j = 0; j < NUM_BULLET_SPHERES; ++j) {
//        BulletVTKObject* sphere = spheres[j];
//        spheres[j] = 0;
//        delete sphere;
//    }
//    //delete collision shapes
////    for (int j = 0; j < collisionShapes.size(); j++)
//    for (int j = 0; j < 2; j++) // because we use the same collision shape
//        // for all spheres
//    {
//        btCollisionShape* shape = collisionShapes[j];
//        collisionShapes[j] = 0;
//        delete shape;
//    }

    //delete dynamics world
    delete dynamics_world;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

    //next line is optional: it will be cleared by the destructor when the array goes out of scope
//    collisionShapes.clear();
}

void TaskNeedle::UpdateGripperLinksPose(const KDL::Frame pose,
    const double grip_angle,
    const std::vector<std::vector<double> > gripper_link_dims,
    BulletVTKObject *link_objects[]
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


