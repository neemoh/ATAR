//
// Created by Andrea on 28/07/2017.
//

#include "TaskBulletTest.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>



TaskBulletTest::TaskBulletTest(const std::string mesh_files_dir,
                       const bool show_ref_frames, const bool biman,
                       const bool with_guidance)
    :
    VTKTask(show_ref_frames, biman, with_guidance, 0) ,
    time_last(ros::Time::now())
{



    InitBullet();

    double *pose;
    double density = 120000;
    double friction = 0.5;
    //std::vector<int> first_path = {1, 3, 2, 0};
    //std::vector<int> second_path = {3, 1, 0, 2};
    //std::vector<int> third_path = {2, 1, 0, 3};
    //index.push_back(first_path);
    //index.push_back(second_path);
    //index.push_back(third_path);
    //target = std::rand() % 3;
    task_state = TaskState::Idle;
    cam_position = {0.08884, 0.24565, 0.13583};
    focal_point = {-0.04443, -0.57915, -0.413638};
    direction = (focal_point-cam_position);
    direction = direction/direction.Norm();
    colors.push_back(Green);
    colors.push_back(Yellow);
    colors.push_back(Blue);

    // -------------------------------------------------------------------------
    // Create static target objects

    std::vector<double> _dim = {1};
    //
    //rot.DoRotX(M_PI);
    //rot.DoRotZ(-M_PI/180*105);
    //double x, y, z, w;
    //rot.GetQuaternion(x, y, z, w);
    //
    for (int i = 0; i < planes_number; i++) {

        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("arrowplane")
                       << std::string(".obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        pose = new double[7] {0, 0, 0, 0, 0, 0, 1};

        plane[i] = new BulletVTKObject(ObjectShape::MESH,
                                      ObjectType::DYNAMIC, _dim, pose, 0.0,
                                      &mesh_file_dir_str, friction);

        dynamicsWorld->addRigidBody(plane[i]->GetBody());
        actors.push_back(plane[i]->GetActor());
        double colour[3] = {colors[i][0], colors[i][1], colors[i][2]};
        plane[i]->GetActor()->GetProperty()->SetColor(colour);
        plane[i]->GetActor()->GetProperty()->SetOpacity(0.0);
    }
    //
    //// -------------------------------------------------------------------------
    //// Arrow for Idle
    //
    //std::stringstream input_file_dir;
    //input_file_dir << mesh_files_dir << std::string("arrow")
    //               <<std::string(".obj");
    //std::string mesh_file_dir_str = input_file_dir.str();
    //
    //rot.DoRotX(-M_PI/2);
    //rot.GetQuaternion(arrow_x, arrow_y, arrow_z, arrow_w);
    //
    //pose = new double[7] {0, 0, 0, 0, 0, 0, 1};
    //
    //arrow = new BulletVTKObject(ObjectShape::MESH,
    //                              ObjectType::DYNAMIC, _dim, pose, 0.0,
    //                              &mesh_file_dir_str, friction);
    //
    //dynamicsWorld->addRigidBody(arrow->GetBody());
    //actors.push_back(arrow->GetActor());
    //arrow->GetActor()->GetProperty()->SetColor(Green_Arrow);
    //arrow->GetActor()->GetProperty()->SetOpacity(1);


    // -------------------------------------------------------------------------
    // Create kinematic pointer

    friction = 0.1;

    pointer_posit = cam_position + (0.05 + 0.16)*direction;
    pose = new double[7] {pointer_posit[0], pointer_posit[1], pointer_posit[2],
    0, 0, 0, 1};
    kine_dim = {0.005, 4*0.007};

    std::stringstream input_file_dir;
    input_file_dir << mesh_files_dir << std::string("orientation_arrow")
                   << std::string(".obj");
    std::string mesh_file_dir_str = input_file_dir.str();

    kine_p=
        new BulletVTKObject(
            ObjectShape::MESH, ObjectType::KINEMATIC, kine_dim, pose,
            0.0, &mesh_file_dir_str, friction);

    dynamicsWorld->addRigidBody(kine_p->GetBody());
    kine_p->GetActor()->GetProperty()->SetColor(0.6314, 0.0, 0.0);
    actors.push_back(kine_p->GetActor());

    // Direction of movement

    movement =  {pointer_posit[0] - starting_point*direction[0],
        pointer_posit[1] - starting_point*direction[1],
        pointer_posit[2] - starting_point*direction[2] + 0.05};
    movement = movement/movement.Norm();
}


//------------------------------------------------------------------------------
void TaskBulletTest::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void TaskBulletTest::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskBulletTest::UpdateActors() {

    //-----------------POINTER: update position

    // Tool
    // KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);
    //
    //KDL::Vector box_posit = tool_pose * KDL::Vector( -0.0, -0.0, -0.03+0.01);
    //
    //double x, y, z, w;
    //tool_pose.M.GetQuaternion(x,y,z,w);
    //double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2],x,y,z,w};
    //kine_box->SetKinematicPose(box_pose);


    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);
    double x, y, z, w;
    tool_pose.M.GetQuaternion(x, y, z, w);
    double pointer_pose[7] = {
        pointer_posit[0], pointer_posit[1], pointer_posit[2], x, y, z, w
    };
    kine_p->SetKinematicPose(pointer_pose);

    // Move the plane towards the user
    double pose[7] = {
        pointer_posit[0] + (starting_point - count) * direction[0],
        pointer_posit[1] + (starting_point - count) * direction[1],
        pointer_posit[2] + (starting_point - count) * direction[2],
        0, 0, 0, 1};

    plane[index]->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(pose));
    KDL::Vector point = {pointer_pose[0], pointer_pose[1], pointer_pose[2]};
    KDL::Vector center = {
        plane[index]->GetActor()->GetCenter()[0]
        , plane[index]->GetActor()->GetCenter()[1]
        , plane[index]->GetActor()->GetCenter()[2]
    };

    int num = index + 1;
    if (index == 2)
        num = 0;
    double _pose[7] = {
        pose[0] - (point - cam_position).Norm(),
        pose[1] - (point - cam_position).Norm(),
        pose[2] - (point - cam_position).Norm(),
        0, 0, 0, 1
        };
    plane[num]->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(_pose));
    plane[num]->GetActor()->GetProperty()->SetOpacity(1.0);
    if (dot(direction, center - point) <= 0.0 &&
        (center - point).Norm() >= (point - cam_position).Norm()) {
        //plane[index]->GetActor()->GetProperty()->SetOpacity(0.0);
        index = index + 1;
        if (index > 2)
            index = 0;
        count = 0;
    }
    plane[index]->GetActor()->GetProperty()->SetOpacity(1.0);
    count = count + 0.005;

    //--------------------------------
    // step the world
    StepDynamicsWorld();






    //if (task_state == TaskState::Idle){
    //    // Manage the position of the arrow
    //    ArrowManager();
    //    axes_dist=0;
    //}
    //else{
    //    // Find the tool coordinates in the target rf
    //    vtkMatrix4x4* ring_rf = vtkMatrix4x4::New();
    //    ring[index[target][path]]->GetActor()->GetMatrix(ring_rf);
    //    double actual_posit[4] = {pointer_posit[0], pointer_posit[1],
    //        pointer_posit[2], 1};
    //    ring_rf->Invert();
    //    ring_rf->MultiplyPoint(actual_posit, transf);
    //
    //    if (task_state == TaskState::Entry){
    //        // Verify if the tool is crossing the target
    //        CheckCrossing();
    //    }
    //    else if (task_state == TaskState::Exit){
    //
    //        ExitChecking();
    //    }
    //
    //    TaskEvaluation();
    //
    //    // Compute the distance between the cylinder axis and the axis
    //    // orthogonal to the ring and passing through its center
    //    axes_dist=(float) sqrt(pow(transf[0], 2) + pow(transf[2], 2));
    //
    //
    //    // Verify if the ring is touched
    //    if (fabs(ring[index[target][path]]->GetActor()->GetCenter()[0] -
    //        ideal_position[index[target][path]].x()) >= 0.0005){
    //        touch = 1;
    //    }
    //
    //}
}


//------------------------------------------------------------------------------
void TaskBulletTest::ArrowManager() {






    //ring[index[target][path]]->GetActor()->GetProperty()->SetColor(Yellow);
    //
    //KDL::Vector shift(0.0, 0.0, -0.03 + 0.005*sin(var));
    //arrow_posit = rot*shift;
    //
    //double* pose;
    //pose = new double[7]{arrow_posit[0] + ideal_position[index[target][path]].x(),
    //    arrow_posit[1] + ideal_position[index[target][path]].y(),
    //    arrow_posit[2] + ideal_position[index[target][path]].z(),
    //    arrow_x, arrow_y, arrow_z, arrow_w};
    //var = var + 0.05;
    //
    //arrow->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(pose));
    //
    //// Check if the arrow has been approached
    //double* arrow_position;
    //arrow_position = arrow->GetActor()->GetCenter();
    //KDL::Vector arrow_pos;
    //arrow_pos.x(arrow_position[0]);
    //arrow_pos.y(arrow_position[1]);
    //arrow_pos.z(arrow_position[2]);
    //distance = arrow_pos - pointer_posit;
    //double norm_distance = distance.Norm();
    //if (norm_distance <= threshold) {
    //    task_state = TaskState::Entry;
    //    begin = ros::Time::now();
    //    arrow->GetActor()->GetProperty()->SetOpacity(0.0);
    //    var = 0;
    //}
}


//------------------------------------------------------------------------------
void TaskBulletTest::CheckCrossing() {






    //// Verify if the tool is crossing the target
    //if (transf[1] >= -kine_dim[1]/2){
    //cond=1;
    //}
    //
    //// Verify if the tool is crossing the target
    //if (transf[1] >= 0 && transf[1] <= kine_dim[1]/2 &&
    //    pow(transf[0], 2) + pow(transf[2], 2) <= pow
    //        (radii[index[target][path]], 2)){
    //
    //    ring[index[target][path]]->GetActor()->GetProperty()->SetColor(Green);
    //    task_state = TaskState::Exit;
    //    cond=0;
    //}

}

//------------------------------------------------------------------------------
void TaskBulletTest::TaskEvaluation() {





    //// Populate the task state message
    //task_state_msg.task_name = "3D Task";
    //task_state_msg.task_state = (uint8_t)task_state;
    //task_state_msg.number_of_repetition = n_rep;
    //
    //// ******* TO FINISH!!!!!!!!!! *******
    ////task_state_msg.error_condition=cond;
    ////task_state_msg.ring_touch=touch;
    ////************************************
    //
    //task_state_msg.error_field_1 = axes_dist;
    //
    //time = (ros::Time::now() - begin).toSec();
    //task_state_msg.time_stamp = time;
}

//------------------------------------------------------------------------------
void TaskBulletTest::ExitChecking() {






        //if (transf[1] >= -kine_dim[1]/2) {
        //    if (path == 3){
        //        ring[index[target][path]]->GetActor()->GetProperty()->SetColor
        //            (Green);
        //    }
        //    else {
        //        ring[index[target][path
        //            + 1]]->GetActor()->GetProperty()->SetColor(Yellow);
        //        ring[index[target][path]]->GetActor()->GetProperty()->SetColor(Green);
        //    }
        //}
        //if (transf[1] < -kine_dim[1]/2){
        //    ring[index[target][path]]->GetActor()->GetProperty()->SetColor(1.0, 1.0, 1.0);
        //    path = path + 1;
        //    task_state = TaskState::Entry;
        //    touch = 0;
        //    if (path > 3){
        //        target = std::rand() % 3;
        //        path = 0;
        //        task_state = TaskState::Idle;
        //        n_rep=n_rep+(unsigned char)1;
        //        arrow->GetActor()->GetProperty()->SetOpacity(1);
        //    }
        //}
}


//------------------------------------------------------------------------------
bool TaskBulletTest::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters TaskBulletTest::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState TaskBulletTest::GetTaskStateMsg() {
    return task_state_msg;
}

void TaskBulletTest::ResetTask() {
    ROS_INFO("Repetition completed. Resetting the task.");

}

void TaskBulletTest::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskBulletTest::FindAndPublishDesiredToolPose() {

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





void TaskBulletTest::InitBullet() {

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


void TaskBulletTest::StepDynamicsWorld() {
    ///-----stepsimulation_start-----

    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamicsWorld->stepSimulation(btScalar(time_step), 5);
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


TaskBulletTest::~TaskBulletTest() {

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

    //for (int j = 0; j < rings_number; ++j) {
    //
    //    delete hinges[j];
    //}
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


