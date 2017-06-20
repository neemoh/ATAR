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



    InitBullet();


    BulletVTKObject* spheres[NUM_BULLET_SPHERES];
    BulletVTKObject* cubes[36];
    BulletVTKObject* board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    board_dimensions[0]  = 0.18;
    board_dimensions[1]  = 0.14;
    board_dimensions[2]  = 0.1;
    double *pose;

    pose= new double[7] {board_dimensions[0] / 2.45,
                         board_dimensions[1] / 2.78,
                         -board_dimensions[2]/2,
                         0, 0, 0, 1};

    std::vector<double> dim = { board_dimensions[0], board_dimensions[1],
                                board_dimensions[2]};
    board = new BulletVTKObject(ObjectShape::BOX,
                                ObjectType::DYNAMIC, dim, pose, 0.0, 10000,
                                0.0, 0.5);
    board->GetActor()->GetProperty()->SetOpacity(0);

    dynamicsWorld->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());

    // -------------------------------------------------------------------------
    // Create spheres
    int cols = 4;
    int rows = NUM_BULLET_SPHERES/cols;
    double density = 3000; // kg/m3
    double stiffnes = 5000;
    double damping = 1;
    double friction = 2;

    for (int i = 0; i < rows+1; ++i) {

        if(i == rows)
            cols = NUM_BULLET_SPHERES%cols;

        for (int j = 0; j < cols; ++j) {

            double ratio = (double)i/4.0;

            pose = new double[7]{(double)i * 0.02,
                                 0.03,
                                 0.08 + 0.02 * j,
                                 0, 0, 0, 1};

            std::vector<double> dim = {0.01};
            spheres[i*rows+j] =
                    new BulletVTKObject(ObjectShape::SPHERE,
                                        ObjectType::DYNAMIC, dim, pose, density,
                                        stiffnes, damping,
                                        friction);
            delete [] pose;
            spheres[i*rows+j]->GetActor()->GetProperty()->SetColor(
                    0.6 - 0.2*ratio, 0.6 - 0.3*ratio, 0.7 + 0.3*ratio);
            spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecular(0.8);
            spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecularPower(50);

            dynamicsWorld->addRigidBody(spheres[i*rows+j]->GetBody());
            actors.push_back(spheres[i*rows+j]->GetActor());

        }
    }

    // -------------------------------------------------------------------------
    // Create cubes
    rows = 3;
    cols = 4;
    int layers = 1;
    double sides = 0.02;
    stiffnes = 20000;
    damping = 2;
    friction = 0.5;

    for (int k = 0; k < layers; ++k) {
        for (int i = 0; i < rows+1; ++i) {
            for (int j = 0; j < cols; ++j) {

                pose = new double[7] {(double)i * (sides + 0.005),
                                      (double)j * (sides + 0.005),
                                      (double)k * (sides + 0.005) + 0.3,
                                      0, 0, 0, 1};

                std::vector<double> dim = {sides, sides, 2*sides};
                cubes[i*rows+j] = new BulletVTKObject(ObjectShape::BOX,
                                                      ObjectType::DYNAMIC, dim,
                                                      pose, density,
                                                      stiffnes, damping,
                                                      friction);
                delete [] pose;

                double ratio = (double)i/4.0;
                cubes[i*rows+j]->GetActor()->GetProperty()->SetColor(
                        0.6 + 0.1*ratio, 0.3 - 0.3*ratio, 0.7 - 0.3*ratio);
                dynamicsWorld->addRigidBody(cubes[i*rows+j]->GetBody());
                actors.push_back(cubes[i*rows+j]->GetActor());

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
    // Create kinematic box

    pose = new double[7] {0, 0, 0, 0, 0, 0, 1};
    std::vector<double> kine_box_dim = {0.005, 0.005, 2*sides};
    kine_box =
            new BulletVTKObject(ObjectShape::BOX,
                                ObjectType::KINEMATIC, kine_box_dim, pose, 0.0);
    dynamicsWorld->addRigidBody(kine_box->GetBody());
    actors.push_back(kine_box->GetActor());
    kine_box->GetActor()->GetProperty()->SetColor(1., 0.1, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic sphere

    std::vector<double> kine_sph_dim = {0.005};
    kine_sphere_0 =
            new BulletVTKObject(ObjectShape::SPHERE,
                                ObjectType::KINEMATIC, kine_sph_dim, pose, 0.0);
    delete [] pose;
    dynamicsWorld->addRigidBody(kine_sphere_0->GetBody());
    actors.push_back(kine_sphere_0->GetActor());
    kine_sphere_0->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic sphere

    kine_sphere_1 =
            new BulletVTKObject(ObjectShape::SPHERE,
                                ObjectType::KINEMATIC, kine_sph_dim, pose, 0.0);
    delete [] pose;
    dynamicsWorld->addRigidBody(kine_sphere_1->GetBody());
    actors.push_back(kine_sphere_1->GetActor());
    kine_sphere_1->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);

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
void BulletTask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void BulletTask::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void BulletTask::UpdateActors() {

    //--------------------------------
    //box
    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    KDL::Vector box_posit = tool_pose * KDL::Vector( 0.0 , 0.0, -0.03);

    double x, y, z, w;
    tool_pose.M.GetQuaternion(x,y,z,w);
    double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2],x,y,z,w};
    kine_box->SetKinematicPose(box_pose);


    //--------------------------------
    //sphere 0
    double grip_posit = (*gripper_position[0]);

    KDL::Vector gripper_pos = KDL::Vector( 0.0, (1+grip_posit)* 0.01, 0.01);
    gripper_pos = tool_pose * gripper_pos;

    double sphere_0_pose[7] = {
        gripper_pos[0],
        gripper_pos[1],
        gripper_pos[2],
        0,0,0,1};
    kine_sphere_0->SetKinematicPose(sphere_0_pose);


    //--------------------------------
    //sphere 1
    gripper_pos = KDL::Vector( 0.0, -(1+grip_posit)* 0.01, 0.01);
    gripper_pos = tool_pose.p + tool_pose.M * gripper_pos;

    double sphere_1_pose[7] = {
        gripper_pos[0],
        gripper_pos[1],
        gripper_pos[2],
        0,0,0,1};
    kine_sphere_1->SetKinematicPose(sphere_1_pose);


    //--------------------------------
    // step the world
    StepDynamicsWorld();

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

}

void BulletTask::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

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





void BulletTask::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,
                                                overlappingPairCache, solver,
                                                collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, 0, -10));


}


void BulletTask::StepDynamicsWorld() {
    ///-----stepsimulation_start-----

    dynamicsWorld->stepSimulation(1.f / 70.f, 40);

//    //print positions of all objects
//    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
//    {
//        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
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


BulletTask::~BulletTask() {

    ROS_INFO("Destructing Bullet task: %d",
             dynamicsWorld->getNumCollisionObjects());
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
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
    delete dynamicsWorld;

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


