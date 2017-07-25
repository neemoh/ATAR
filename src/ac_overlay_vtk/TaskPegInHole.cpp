//
// Created by nima on 13/06/17.
//

#include "TaskPegInHole.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#define _USE_MATH_DEFINES


TaskPegInHole::TaskPegInHole(const std::string mesh_files_dir,
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
        board = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose, 0.0, NULL,
            friction
        );
//    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.8, 0.3, 0.1);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject *floor = new BulletVTKObject(
        ObjectShape::STATICPLANE, ObjectType::DYNAMIC,
        floor_dims, dummy_pose, 0.0, NULL
    );
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create cylinders
    {
        uint cols = 4;
        uint rows = 3;
        float density = 50000; // kg/m3
        float friction = 2.2;
        BulletVTKObject *cylinders[cols * rows];
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {

                std::vector<double> dim = {0.003, 0.03};

                double q3 = 0;
                double q4 = 1;
                if (j < cols / 2) {
                    q3 = 0.70711;
                    q4 = 0.70711;
                }
                double pose[7]{
                    (double) i * 4 * dim[0] + (double) j * dim[0] / 2, 0.06, 0.1
                        + dim[1] * 1.5 * (double) j, 0, 0, q3, q4
                };

                cylinders[i * rows + j] =
                    new BulletVTKObject(
                        ObjectShape::CYLINDER, ObjectType::DYNAMIC, dim, pose,
                        density, NULL, friction
                    );
                double ratio = (double) i / 4.0;
                cylinders[i * rows + j]->GetActor()->GetProperty()->SetColor(
                    0.6 - 0.2 * ratio, 0.6 - 0.3 * ratio, 0.7 + 0.3 * ratio
                );
                cylinders[i * rows + j]->GetActor()->GetProperty()->SetSpecular(
                    0.8
                );
                cylinders[i * rows
                    + j]->GetActor()->GetProperty()->SetSpecularPower(50);

                dynamics_world->addRigidBody(
                    cylinders[i * rows + j]->GetBody());
                actors.push_back(cylinders[i * rows + j]->GetActor());

            }
        }
    }
    // -------------------------------------------------------------------------
    // Create cubes
    {
        uint rows = 3;
        uint cols = 2;
        int layers = 3;
        BulletVTKObject *cubes[layers * rows * cols];

        double sides = 0.006;
        float density = 50000; // kg/m3
        float friction = 12;

        for (int k = 0; k < layers; ++k) {
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {

                    double pose[7]{
                        (double) i * 2.2 * sides + 0.1, (double) j * 2.2 * sides
                            + 0.05, (double) k * 4 * sides + 0.05, 0, 0, 0, 1
                    };

                    std::vector<double> dim = {sides, sides, sides};
                    cubes[i * rows + j] = new BulletVTKObject(
                        ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose,
                        density,
                        NULL, friction
                    );
                    double ratio = (double) i / 4.0;
                    cubes[i * rows + j]->GetActor()->GetProperty()->SetColor(
                        0.6 + 0.1 * ratio, 0.3 - 0.3 * ratio, 0.7 - 0.3 * ratio
                    );

//                cubes[i*rows+j]->GetBody()->setContactStiffnessAndDamping(
//                        (float) stiffnes, (float) damping);
                    dynamics_world->addRigidBody(
                        cubes[i * rows + j]->GetBody());
                    actors.push_back(cubes[i * rows + j]->GetActor());

                }
            }
        }
    }

    {
//        std::vector<double> dim = {sides, sides, sides};
//        cubes[i*rows+j] = new BulletVTKObject(ObjectShape::BOX,
//                                              ObjectType::DYNAMIC, dim,
//                                              pose, 0.2), stiffnes, damping;

    }

    // -------------------------------------------------------------------------
    //// Create mesh
    {
        double pose[7]{0.09, 0.06, 0.08, 0.7, 0, 0.7, 0};
        std::vector<double> _dim = {0.002};

        float friction = 3;
        float density = 50000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("needle_L3cm_d3mm.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        needle_mesh = new
            BulletVTKObject(
            ObjectShape::MESH,
            ObjectType::DYNAMIC, _dim, pose, density,
            &mesh_file_dir_str,
            friction
        );

        dynamics_world->addRigidBody(needle_mesh->GetBody());
        actors.push_back(needle_mesh->GetActor());
        needle_mesh->GetActor()->GetProperty()->SetColor(0.8f, 0.8f, 0.8f);
    }

    {
        double pose[7]{0.07, 0.06, 0.08, 0.7, 0, 0.7, 0};
        std::vector<double> _dim = {0.002};

        float friction = 20;
        float density = 50000; // kg/m3
        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("ring.obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        ring_mesh = new
            BulletVTKObject(
            ObjectShape::MESH,
            ObjectType::DYNAMIC, _dim, pose, density,
            &mesh_file_dir_str,
            friction
        );

        dynamics_world->addRigidBody(ring_mesh->GetBody());
        actors.push_back(ring_mesh->GetActor());
        ring_mesh->GetActor()->GetProperty()->SetColor(0.4f, 0.3f, 0.3f);
        //ring_mesh->GetBody()->setContactStiffnessAndDamping(5000, 10);
    }
    // -------------------------------------------------------------------------
    // Create kinematic box
    //friction = 100;
    {
        double pose[7]{0, 0, 0, 0, 0, 0, 1};
        std::vector<double> kine_box_c_dim = {0.002, 0.002, 0.01};
        float friction = 10;
        jaw_links[0] =
            new BulletVTKObject(
                ObjectShape::BOX, ObjectType::KINEMATIC, kine_box_c_dim, pose,
                0.0,
                NULL, friction
            );
        dynamics_world->addRigidBody(jaw_links[0]->GetBody());
        actors.push_back(jaw_links[0]->GetActor());
        jaw_links[0]->GetActor()->GetProperty()->SetColor(0.6f, 0.2f, 0.3f);
    }
    // -------------------------------------------------------------------------
    // Create kinematic jaw
    float jaw_density = 0; // kg/m3
    float jaw_friction = 10;
    double jaw_pose[7]{0, 0, 0, 0, 0, 0, 1};
    jaw_link_dims =
        {   {0.003, 0.003, 0.005}
          , {0.004, 0.001, 0.009}
          , {0.004, 0.001, 0.009}
          , {0.004, 0.001, 0.007}
          , {0.004, 0.001, 0.007}};

   // ----
    // -------------------------------------------------------------------------
    // Create kinematic box
    //friction = 100;
    {
        for (int i = 0; i < 5; ++i) {
            jaw_links[i] =
                new BulletVTKObject(
                    ObjectShape::BOX, ObjectType::KINEMATIC, jaw_link_dims[i], jaw_pose,
                    jaw_density,
                    NULL, jaw_friction
                );
            dynamics_world->addRigidBody(jaw_links[i]->GetBody());
            actors.push_back(jaw_links[i]->GetActor());
            jaw_links[i]->GetActor()->GetProperty()->SetColor(0.35f, 0.4f, 0.4f);
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

    }


//------------------------------------------------------------------------------
void TaskPegInHole::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void TaskPegInHole::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskPegInHole::UpdateActors() {

    //--------------------------------
    //box
    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    KDL::Vector box_posit = tool_pose * KDL::Vector( 0.0 , 0.0,
                                                     -jaw_link_dims[0][2]/2);

    double x, y, z, w;
    tool_pose.M.GetQuaternion(x,y,z,w);
    double link0_pose[7] = {box_posit[0], box_posit[1], box_posit[2],x,y,z,w};
    jaw_links[0]->SetKinematicPose(link0_pose);


    //--------------------------------
    // first gripper
    double grip_posit = (*gripper_position[0]);
    double theta_min=20*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    KDL::Rotation tool_p_1=tool_pose.M;
    tool_p_1.DoRotX(-grip_angle);
    KDL::Vector angular_position_1 = tool_pose.p+ tool_p_1 *
        KDL::Vector( 0.0, 0.0, jaw_link_dims[1][2]/2);
    tool_p_1.GetQuaternion(x, y, z, w);

    double link2_pose[7] = {
        angular_position_1[0],
        angular_position_1[1],
        angular_position_1[2],
        x, y, z, w};

    jaw_links[1]->SetKinematicPose(link2_pose);

    //--------------------------------


    KDL::Rotation tool_p_2=tool_pose.M;
    tool_p_2.DoRotX(grip_angle);
    KDL::Vector angular_position_2 = tool_pose.p+ tool_p_2 *
        KDL::Vector( 0.0, 0.0, jaw_link_dims[2][2]/2);
    tool_p_2.GetQuaternion(x, y, z, w);

    double link3_pose[7] = {
        angular_position_2[0],
        angular_position_2[1],
        angular_position_2[2],
        x, y, z, w};

    jaw_links[2]->SetKinematicPose(link3_pose);


    //--------------------------------

    KDL::Vector linear_position_1  =      tool_pose.p
        + tool_p_1 * KDL::Vector( 0.0, 0.0, jaw_link_dims[1][2])
        + tool_pose.M * KDL::Vector( 0.0, 0.0, jaw_link_dims[3][2]/2);

    tool_pose.M.GetQuaternion(x,y,z,w);
    double link4_pose[7] = {
        linear_position_1[0],
        linear_position_1[1],
        linear_position_1[2],
        x, y, z, w};

    jaw_links[3]->SetKinematicPose(link4_pose);


    //--------------------------------
    KDL::Vector linear_position_2 =tool_pose.p+ tool_p_2*
        KDL::Vector( 0.0, 0.0, jaw_link_dims[2][2])
        + tool_pose.M * KDL::Vector( 0.0, 0.0, jaw_link_dims[4][2]/2);


    double link5_pose[7] = {
        linear_position_2[0],
        linear_position_2[1],
        linear_position_2[2],
        x, y, z, w};

    jaw_links[4]->SetKinematicPose(link5_pose);


    //--------------------------------
    // step the world
    StepDynamicsWorld();

}




//------------------------------------------------------------------------------
bool TaskPegInHole::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters TaskPegInHole::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState TaskPegInHole::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskPegInHole::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskPegInHole::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskPegInHole::FindAndPublishDesiredToolPose() {

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





void TaskPegInHole::InitBullet() {

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


}


void TaskPegInHole::StepDynamicsWorld() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();

//    //print positions of all objects
//    for (int j = dynamics_world->getNumCollisionObjects() - 1; j >= 0; j--)
//    {
//        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[j];
//        btRigidBody* body_ = btRigidBody::upcast(obj);
//        btTransform trans;
//        if (body_ && body_->getMotionState())
//        {
//            body_->getMotionState()->getWorldTransform(trans);
//        }
//        else
//        {
//            trans = obj->getWorldTransform();
//        }
//
//            heights[j] = trans.getOrigin().z();
//        heights2[j] = actors[j]->GetMatrix()->Element[2][3];
//    }

}


TaskPegInHole::~TaskPegInHole() {

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


