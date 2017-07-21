//
// Created by nima on 21/07/17.
//

#include "TaskHook.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#define _USE_MATH_DEFINES


TaskHook::TaskHook(const std::string mesh_files_dir,
                       const bool show_ref_frames, const bool biman,
                       const bool with_guidance)
    :
    VTKTask(show_ref_frames, biman, with_guidance, 0),
    time_last(ros::Time::now())
{

    InitBullet();

    BulletVTKObject* board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    //board_dimensions[0]  = 0.18;
    //board_dimensions[1]  = 0.14;
    //board_dimensions[2]  = 0.1;

    board_dimensions[0]  = 0.28;
    board_dimensions[1]  = 0.24;
    board_dimensions[2]  = 0.1;
    double *pose;
    //double stiffnes = 1000;
    //double damping = 20;
    double friction = 0.5;

    pose= new double[7] {board_dimensions[0] / 2.45,
        board_dimensions[1] / 2.78,
        -board_dimensions[2]/2,
        0, 0, 0, 1};

    std::vector<double> dim = { board_dimensions[0], board_dimensions[1],
        board_dimensions[2]};
    board = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose, 0.0, NULL, friction
    );
//    board->GetActor()->GetProperty()->SetOpacity(0.05);
    board->GetActor()->GetProperty()->SetColor(0.6, 0.5, 0.5);

    dynamicsWorld->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());

    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject* floor= new BulletVTKObject(
        ObjectShape::STATICPLANE, ObjectType::DYNAMIC,
        floor_dims, dummy_pose, 0.0, NULL);
    dynamicsWorld->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create cylinders
    int cols = 4;
    int rows = 1;
    double density = 50000; // kg/m3
    friction = 2.2;
    BulletVTKObject* cylinders[cols*rows];
    for (int i = 0; i < rows; ++i) {

        for (int j = 0; j < cols; ++j) {

            std::vector<double> dim = {0.002, 0.035};

            pose = new double[7]{
                0.03 + (double)j * 0.028,
                0.09 ,
                dim[1]/2,
                0, 0.70711, 0.70711, 0.0};

            cylinders[i*rows+j] =
                new BulletVTKObject(
                    ObjectShape::CYLINDER, ObjectType::DYNAMIC, dim, pose,
                    0.0, NULL, friction
                );
            delete [] pose;
            double ratio = (double)i/4.0;
            cylinders[i*rows+j]->GetActor()->GetProperty()->SetColor(
                0.6 - 0.2*ratio, 0.6 - 0.3*ratio, 0.7 + 0.3*ratio);
            cylinders[i*rows+j]->GetActor()->GetProperty()->SetSpecular(0.8);
            cylinders[i*rows+j]->GetActor()->GetProperty()->SetSpecularPower(50);

            dynamicsWorld->addRigidBody(cylinders[i*rows+j]->GetBody());
            actors.push_back(cylinders[i*rows+j]->GetActor());

        }
    }


    // -------------------------------------------------------------------------
    // ROD
    std::vector<double> rod_dim = {0.002, 0.1};
    pose = new double[7] {0.10, 0.0, 0.03, 0.70711, 0.70711, 0.0, 0.0};

    BulletVTKObject rod = BulletVTKObject(
            ObjectShape::CYLINDER, ObjectType::DYNAMIC, rod_dim, pose,
            0.0, NULL, friction
        );
    dynamicsWorld->addRigidBody(rod.GetBody());
    actors.push_back(rod.GetActor());
    delete [] pose;
    // -------------------------------------------------------------------------

//    rows = 3;
//    cols = 2;
//    int layers = 3;
//    BulletVTKObject* cubes[layers *rows *cols];
//
//    double sides = 0.006;
//    //density = 5; // kg/m3
//    //stiffnes = 20000;
//    //damping = 0.9;
//    //friction = 100.1;
//
//    for (int k = 0; k < layers; ++k) {
//        for (int i = 0; i < rows; ++i) {
//            for (int j = 0; j < cols; ++j) {
//
//                pose = new double[7] {(double)i * 2.2*sides + 0.1,
//                    (double)j * 2.2*sides  + 0.05,
//                    (double)k * 4*sides  + 0.05,
//                    0, 0, 0, 1};
//
//                std::vector<double> dim = {sides, sides, sides};
//                cubes[i*rows+j] = new BulletVTKObject(
//                    ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose, density,
//                    NULL, friction
//                );
//                delete [] pose;
//
//                double ratio = (double)i/4.0;
//                cubes[i*rows+j]->GetActor()->GetProperty()->SetColor(
//                    0.6 + 0.1*ratio, 0.3 - 0.3*ratio, 0.7 - 0.3*ratio);
//
////                cubes[i*rows+j]->GetBody()->setContactStiffnessAndDamping(
////                        (float) stiffnes, (float) damping);
//                dynamicsWorld->addRigidBody(cubes[i*rows+j]->GetBody());
//                actors.push_back(cubes[i*rows+j]->GetActor());
//
//            }
//        }
//    }


    // -------------------------------------------------------------------------
    //// Create mesh

    pose = new double[7] {0.09, 0.07, 0.08, 0.7, 0, 0.7, 0};
    //    BulletVTKObject *mesh;
    std::stringstream input_file_dir;
    input_file_dir << mesh_files_dir << std::string("ring.obj");
    std::string mesh_file_dir_str = input_file_dir.str();
    density = 20000;
    ring_mesh = new
        BulletVTKObject(ObjectShape::MESH,
                        ObjectType::DYNAMIC, dim, pose, density,
                        &mesh_file_dir_str,
                        friction);
    delete [] pose;
    dynamicsWorld->addRigidBody(ring_mesh->GetBody());
    actors.push_back(ring_mesh->GetActor());
    ring_mesh->GetActor()->GetProperty()->SetColor(4., 0.2, 0.5);

    // -------------------------------------------------------------------------
    //// Create smallRING mesh
    size_t n_rings = 3;
    BulletVTKObject* rings[n_rings];
    input_file_dir.str("");
    input_file_dir << mesh_files_dir << std::string("ring_D2cm_D5mm.obj");
    mesh_file_dir_str = input_file_dir.str();

    for (int l = 0; l < n_rings; ++l) {

        pose = new double[7] {0.07 + (double)l *0.01, 0.0, 0.03, 0.70711,
            0.70711, 0.0, 0.0};

        rings[l] = new
            BulletVTKObject(ObjectShape::MESH,
                            ObjectType::DYNAMIC, dim, pose, density,
                            &mesh_file_dir_str,
                            friction);
        delete [] pose;
        dynamicsWorld->addRigidBody(rings[l]->GetBody());
        actors.push_back(rings[l]->GetActor());
        rings[l]->GetActor()->GetProperty()->SetColor(0., 0.5, 0.6);

    }

    // -------------------------------------------------------------------------
    //// Create hook mesh

    pose = new double[7] {0.09, 0.07, 0.08, 0.7, 0, 0.7, 0};
    input_file_dir.str("");
    input_file_dir << mesh_files_dir << std::string("hook.obj");
    mesh_file_dir_str = input_file_dir.str();

    hook_mesh = new
        BulletVTKObject(ObjectShape::MESH,
                        ObjectType::KINEMATIC, dim, pose, density,
                        &mesh_file_dir_str,
                        friction);
    delete [] pose;
    dynamicsWorld->addRigidBody(hook_mesh->GetBody());
    actors.push_back(hook_mesh->GetActor());
    hook_mesh->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);


    // -------------------------------------------------------------------------
    //// Create hook mesh
    {
        std::vector<double> dim = {0.002, 0.03};
        tool_cyl = new BulletVTKObject(
            ObjectShape::CYLINDER, ObjectType::KINEMATIC, dim, pose,
            0.0, NULL, friction);
        dynamicsWorld->addRigidBody(tool_cyl->GetBody());
        actors.push_back(tool_cyl->GetActor());
        tool_cyl->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);
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
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskHook::UpdateActors() {

    //--------------------------------
    //box
    {
        KDL::Frame tool_pose = (*tool_current_pose_kdl[1]);

        KDL::Vector box_posit = tool_pose * KDL::Vector(0.0, 0.0, 0);

        double x, y, z, w;
        tool_pose.M.GetQuaternion(x, y, z, w);
        double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2], x, y, z, w};
        hook_mesh->SetKinematicPose(box_pose);

    }
    //--------------------------------
    //box
    {
        KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);
        KDL::Vector box_posit = tool_pose * KDL::Vector(0.0, 0.0, 0);
        double x, y, z, w;
        tool_pose.M.GetQuaternion(x, y, z, w);
        double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2], x, y, z, w};
        tool_cyl->SetKinematicPose(box_pose);

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
custom_msgs::ActiveConstraintParameters TaskHook::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
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

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,
                                                overlappingPairCache, solver,
                                                collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, 0, -10));


}


void TaskHook::StepDynamicsWorld() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamicsWorld->stepSimulation(btScalar(time_step*0.5), 5);
    time_last = ros::Time::now();

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


TaskHook::~TaskHook() {

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


