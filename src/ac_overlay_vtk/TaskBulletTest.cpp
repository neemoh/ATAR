//
// Created by Andrea on 28/07/2017.
//

#include "TaskBulletTest.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>

double Green[3] {0.0, 0.9, 0.03};
double Yellow[3] {1.0, 1.0, 0.0};

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
    double friction = 6;

    // -------------------------------------------------------------------------
    // Create static target objects

    offset = new double[rings_number] {0.012, 0.01, 0.011, 0.014};

    std::vector<double> _dim = {1};

    KDL::Rotation rot;
    rot.DoRotX(M_PI);
    rot.DoRotZ(-M_PI/180*105);
    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);

    // rotation of the hinges
    KDL::Rotation rot1;
    rot1.DoRotZ(M_PI/180*195);
    double _x, _y, _z, _w;
    rot1.GetQuaternion(_x, _y, _z, _w);

    KDL::Vector cam_position(0.118884, 0.27565, 0.14583);
    KDL::Vector focal_point(-0.04443, -0.57915, -0.413638);
    KDL::Vector direction = (focal_point-cam_position);
    direction = direction/direction.Norm();

    kine_pointer_dim = {0.002, 0.002};

    for (int i = 0; i<rings_number; i++){


        std::stringstream input_file_dir;
        input_file_dir << mesh_files_dir << std::string("3Dring")
                       << i+1 <<std::string(".obj");
        std::string mesh_file_dir_str = input_file_dir.str();

        KDL::Vector ring_pos;
        ring_pos = cam_position +  ((double)i* 0.05 + 0.16 +
            sin((double)i*M_PI/3)*(0.05/4))*direction;

        pose = new double[7] {ring_pos.x(),
            ring_pos.y(),
            ring_pos.z(),
            x, y, z, w};

        ideal_position[i].x(pose[0]);
        ideal_position[i].y(pose[1]);
        ideal_position[i].z(pose[2]);

        ring[i] = new BulletVTKObject(ObjectShape::MESH,
                                      ObjectType::DYNAMIC, _dim, pose, density,
                                      &mesh_file_dir_str, friction);
        dynamicsWorld->addRigidBody(ring[i]->GetBody());
        actors.push_back(ring[i]->GetActor());
        ring[i]->GetActor()->GetProperty()->SetColor(0.8f, 0.8f, 0.8f);


        const btVector3 btPivotA(0.f, 0.f, -(0.025f+offset[i])*B_DIM_SCALE);
        btVector3 btAxisA( 1.0f, 0.0f, 0.0f );
        ring[i]->GetBody()->setRollingFriction(2);
        ring[i]->GetBody()->setSpinningFriction(2);
        hinges[i] = new btHingeConstraint( *ring[i]->GetBody(), btPivotA, btAxisA );
        hinges[i]->enableAngularMotor(true, 0 , 0.00015);
        dynamicsWorld->addConstraint(hinges[i]);

        // hinge objects
        pose = new double[7] {ring_pos.x(),
            ring_pos.y(),
            ring_pos.z() + 0.025 + offset[i],
            _x, _y, _z, _w};

        hinge_cyl[i] = new BulletVTKObject(ObjectShape::CYLINDER,
                                           ObjectType::DYNAMIC,
                                           kine_pointer_dim, pose, 0.0, NULL,
                                           friction);

        dynamicsWorld->addRigidBody(hinge_cyl[i]->GetBody());
        hinge_cyl[i]->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.5);
        actors.push_back(hinge_cyl[i]->GetActor());
    }

    ring[target]->GetActor()->GetProperty()->GetColor(color);
    ring[target]->GetActor()->GetProperty()->SetColor(Yellow);

    // -------------------------------------------------------------------------
    // Create kinematic pointer

    friction = 50.1;

    pose = new double[7] {0, 0, 0, 0, 0, 0, 1};
    kine_pointer_dim = kine_dim;
    kine_p=
        new BulletVTKObject(
            ObjectShape::CYLINDER, ObjectType::KINEMATIC, kine_pointer_dim, pose,
            0.0,
            NULL, friction
        );
    dynamicsWorld->addRigidBody(kine_p->GetBody());
    kine_p->GetActor()->GetProperty()->SetColor(1.0, 0.1, 0.1);
    actors.push_back(kine_p->GetActor());
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

    //-----------------POINTER: update position on the upper plane

    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    pointer_posit = tool_pose * KDL::Vector( -0.0, -0.0, -0.03+0.01);
    KDL::Rotation rot;
    rot.DoRotZ(M_PI/180*105);
    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);
    //double x, y, z, w;
    //tool_pose.M.GetQuaternion(x,y,z,w);
    double pointer_pose[7] = {pointer_posit[0], pointer_posit[1], pointer_posit[2],
        x,y,z,w};
    kine_p->SetKinematicPose(pointer_pose);

    //-----------------UPDATE right GRIPPER

    // map gripper value to an angle
    //double grip_posit = (*gripper_position[0]); //belonging to -0.5 - 1.55 interval
    //
    //if(grip_posit >threshold) cond=1;
    //
    ////if(grip_posit <threshold && cond==1){
    ////    cond=0;
    ////}

    //--------------------------------
    // step the world
    StepDynamicsWorld();

    TaskEvaluation();

}


//------------------------------------------------------------------------------
void TaskBulletTest::TaskEvaluation() {

    //// Compute the distance of the target from the ideal value
    //target_position = ring[random_ring]->GetActor()->GetCenter();
    //KDL::Vector id_pos;
    //id_pos.x(target_position[0]);
    //id_pos.y(target_position[1]);
    //id_pos.z(target_position[2]);
    //distance = id_pos - pointer_posit;
    //double norm_distance = distance.Norm();

    // Find the tool coordinates in the target rf
    vtkMatrix4x4* ring_rf = vtkMatrix4x4::New();
    ring[target]->GetActor()->GetMatrix(ring_rf);
    double transf[4];
    double actual_posit[4] = {pointer_posit[0], pointer_posit[1],
        pointer_posit[2], 1};
    ring_rf->Invert();
    ring_rf->MultiplyPoint(actual_posit, transf);
    std::cout << "x:" << transf[0] << "y:" << transf[1] << "z:" << transf[2]
              << std::endl << kine_dim[2]+1 <<
                                                                      std::endl;

    // Verify if the tool is crossing the target
    if (transf[1] >= -kine_dim[2]/2 &&
        pow(transf[0], 2) + pow(transf[2], 2) <= pow(offset[target], 2)){
        std::cout << "crossing" << std::endl;
        ring[target]->GetActor()->GetProperty()->SetColor(Green);
        //if (transf[2] <= 0){
        //    ring[target]->GetActor()->GetProperty()->SetColor(color);
        //    target = rand() % (rings_number) + 1;
        //    ring[target]->GetActor()->GetProperty()->SetColor(Yellow);
        //}
    }
    else ring[target]->GetActor()->GetProperty()->SetColor(Yellow);
    std::cout << "not crossing" << std::endl;


    // Update new ring target saving the actual color of the target and
    // changing the color of the new one
    //random_ring = rand() % (rings_number) + 1;
    //ring[random_ring]->GetActor()->GetProperty()->GetColor(color);
    //ring[random_ring]->GetActor()->GetProperty()->SetColor(Green);

    //// Computing Area of Correct Placement (ACP), i.e. the fraction of the area
    //// of the pointer superimposed to the target
    //
    //// Need to find theta to define the circular sector
    //double temp_x = board_dimensions[0] / 2 - fabs(distance.x());
    //double temp_y = board_dimensions[1] / 2 - fabs(distance.y());
    //
    //if (temp_x>(-kine_pointer_dim[0]) && temp_y>(-kine_pointer_dim[0])) {
    //    double chord_length =
    //        sqrt(
    //            pow(sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_x, 2.0))
    //                    +temp_y,2.0) +
    //            pow(sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_y, 2.0))
    //                    +temp_x,2.0));
    //    double theta =
    //        2 * asin((chord_length / (2 * kine_pointer_dim[0])));
    //    std::cout << "theta: " << theta << std::endl;
    //    // THETA ALWAYS NAN, DON'T KNOW WHY
    //
    //    // Composition of Surfaces
    //    double Asector = kine_pointer_dim[0]*kine_pointer_dim[0]*theta/2;
    //    double Arectangle = temp_x * temp_y;
    //    std::cout << "A rect: " << Arectangle << std::endl;
    //
    //    double Atriangle1 = temp_x *
    //        sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_x, 2.0)) / 2;
    //    std::cout << "A triang1: " << Atriangle1 << std::endl;
    //
    //    double Atriangle2 = temp_y *
    //        sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_y, 2.0)) / 2;
    //    std::cout << "A triang2: " << Atriangle2 << std::endl;
    //
    //    double Aoverlap = Asector + Arectangle + Atriangle1 + Atriangle2;
    //    std::cout << "A overlap: " <<Aoverlap << std::endl;
    //
    //    // ACP
    //    ACP = (Aoverlap / (M_PI * pow(kine_pointer_dim[0], 2.0))) * 100;
    //} else
    //    ACP = 0;
    //
    //// Cycle accessed only when the task begins (after an initial grip of
    //// start).
    //// It generates the message at each trial
    //if (start==1){
    //    task_state_msg.task_name = "3D Test";
    //    time = (ros::Time::now() - begin).toSec();
    //    task_state_msg.time_stamp = time;
    //    task_state_msg.number_of_repetition=rep;
    //    rep = rep + uint8_t (1);
    //    task_state_msg.error_field_1 = norm_distance;
    //    task_state_msg.task_state = uint8_t (correct);
    //    task_state_msg.error_field_2 = ACP;
    //}
    //// Initialization
    //start = 1;
    //begin = ros::Time::now();
    //
    //std::cout << task_state_msg << std::endl;
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

    for (int j = 0; j < rings_number; ++j) {

        delete hinges[j];
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


