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


    // -------------------------------------------------------------------------
    // Create a cube for the board

    board_dimensions[0]  = 0.18;
    board_dimensions[1]  = 0.14;
    board_dimensions[2]  = 0.01;

    double pose[7] = {board_dimensions[0] / 2.45,
                      board_dimensions[1] / 2.78,
                      -board_dimensions[2]/2,
                      0, 0, 0, 1};

    std::vector<double> dim = { board_dimensions[0], board_dimensions[1],
                                board_dimensions[2]};
    board = new BulletVTKObject(ObjectShape::BOX,
                                ObjectType::DYNAMIC, dim, pose, 0.0);

    dynamicsWorld->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());

    // -------------------------------------------------------------------------
    // Create spheres
    int cols = 4;

    int rows = NUM_BULLET_SPHERES/cols;
    for (int i = 0; i < rows+1; ++i) {

        if(i == rows)
            cols = NUM_BULLET_SPHERES%cols;

        for (int j = 0; j < cols; ++j) {

        double ratio = (double)i/4.0;

        double pose[7] = {(double)i * 0.02,
                          0.0,
                          0.04 + 0.02 * j,
                          0, 0, 0, 1};

        std::vector<double> dim = {RAD_SPHERES};
        spheres[i*rows+j] = new BulletVTKObject(ObjectShape::SPHERE,
                                         ObjectType::DYNAMIC, dim, pose, 0.1);

        spheres[i*rows+j]->GetActor()->GetProperty()->SetColor(
                0.6 - 0.2*ratio, 0.6 - 0.3*ratio, 0.7 + 0.3*ratio);
        spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecular(0.8);
        spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecularPower(50);

        dynamicsWorld->addRigidBody(spheres[i*rows+j]->GetBody());
        actors.push_back(spheres[i*rows+j]->GetActor());

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
void BulletTask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

//------------------------------------------------------------------------------
void BulletTask::UpdateActors() {

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

    dynamicsWorld->stepSimulation(1.f / 90.f, 20);

//    //print positions of all objects
//    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
//    {
//        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
//        btRigidBody* body = btRigidBody::upcast(obj);
//        btTransform trans;
//        if (body && body->getMotionState())
//        {
//            body->getMotionState()->getWorldTransform(trans);
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


