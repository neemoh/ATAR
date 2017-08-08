///---------------Created by Andre Eddy ------------------------------------
//******************** 06/08/2017 *******************************************

#include "TaskClutch.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>

double Green[3] {0.0, 0.9, 0.03};
//double Yellow[3] {0.6, 0.802, 0.0};


TaskClutch::TaskClutch(const std::string mesh_files_dir,
                               const bool show_ref_frames, const bool biman,
                               const bool with_guidance)
    :
    VTKTask(show_ref_frames, biman, with_guidance, 0) ,
    time_last(ros::Time::now()) {

    InitBullet();
    task_state = TaskState::Idle;


    BulletVTKObject *board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create the higher level board: set opacity to 0.5 since the lower
    // board has to be visible -> it's the reference

    //board_dimensions[0]  = 0.18;
    //board_dimensions[1]  = 0.14;
    //board_dimensions[2]  = 0.1;

    board_dimensions[0] = 0.12;
    board_dimensions[1] = 0.12;
    board_dimensions[2] = 0.005;
    double *pose;
    double density;
    double stiffnes = 1000;
    double damping = 20;
    double friction = 6;

    KDL::Vector cam_position(0.118884, 0.27565, 0.14583);
    KDL::Vector focal_point(-0.04443, -0.57915, -0.413638);
    KDL::Vector direction = (focal_point-cam_position);
    direction = direction/direction.Norm();

    rot.Identity();
    rot.UnitZ(-direction);
    rot.UnitY((-direction)*rot.UnitX());
    rot.UnitX(rot.UnitY()*rot.UnitZ());

    double x, y, z, w;
    rot.GetQuaternion(x, y, z, w);

    box_pose.x(0);
    box_pose.y(0);
    box_pose.z(0.00245);

    box_pose=rot*box_pose;

    pose = new double[7]{
        box_pose.x() , box_pose.y(), box_pose.z(), x, y, z, w
    };

    std::vector<double> dim = {
        board_dimensions[0]*3, board_dimensions[1]*3, board_dimensions[2]
    };
    board = new BulletVTKObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim,
                                pose, 0.0, 0, friction,
                                NULL);
    board->GetActor()->GetProperty()->SetOpacity(1.0);
    board->GetActor()->GetProperty()->SetColor(0.2549, 0.4117, 0.8823);

    dynamicsWorld->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());



    // -------------------------------------------------------------------------
    //// Create base chessboard (the # of cols and rows is defined in the
    /// header file)

    bool index;
    index = 0; //index just to alternate black and white boxes

    board_dimensions[0] = 0.12 / cols;
    board_dimensions[1] = 0.12 / rows;
    board_dimensions[2] = 0.005;
    height=board_dimensions[2];

    dim = {
        board_dimensions[0], board_dimensions[1],  board_dimensions[2]
    };


    for (int i = 0; i < rows; ++i) {
        index = !(index);
        for (int j = 0; j < cols; ++j) {

            box_pose.x((double) i * dim[0] + dim[0] / 2-0.01);
            box_pose.y((double) j * dim[1] + dim[1] / 2-0.038);
            box_pose.z(dim[2]/2);

            KDL::Vector position;
            position=rot*box_pose;



            pose = new double[7]{
                position.x(), position.y(), position.z(), x, y, z, w
            };

            chessboard[i * rows + j] =
                    new BulletVTKObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                                        dim, pose, 0.0, 0, friction,
                                        NULL);

            delete[] pose;

            if (index == 0) {
                chessboard[i * rows + j]->GetActor()->GetProperty()->SetColor(
                    0.0, 0.0, 0.0
                );
            } else {
                chessboard[i * rows + j]->GetActor()->GetProperty()->SetColor(
                    1.0, 1.0, 1.0
                );
            }

            index = !(index);

            dynamicsWorld->addRigidBody(chessboard[i * rows + j]->GetBody());
            actors.push_back(chessboard[i * rows + j]->GetActor());

        }

        box_n = std::rand() % (cols * rows) +1;


        // -------------------------------------------------------------------------
        // Create kinematic pointer

        stiffnes = 1000;
        damping = 100;
        friction = 50.1;

        pose = new double[7]{0, 0, 0, 0, 0, 0, 1};
        kine_pointer_dim = {2 * 0.0025};
        kine_p =
                new BulletVTKObject(ObjectShape::SPHERE, ObjectType::KINEMATIC,
                                    kine_pointer_dim, pose, 0.0, 0, friction,
                                    NULL);
        dynamicsWorld->addRigidBody(kine_p->GetBody());
        kine_p->GetActor()->GetProperty()->SetColor(1., 0.1, 0.1);
        actors.push_back(kine_p->GetActor());

        //Compute the inverse of rot
        rot_inv=rot.Inverse();
        //ResetTask();
    }
}


//------------------------------------------------------------------------------
void TaskClutch::SetCurrentToolPosePointer(
    KDL::Frame &tool_pose,
    const int tool_id
) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

void TaskClutch::SetCurrentGripperpositionPointer(
    double &grip_position, const int
tool_id
) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskClutch::UpdateActors() {

    if (task_state == TaskState::Idle){

        if (init==1)  {
            chessboard[box_n]->GetActor()->GetProperty()->GetColor(color);
            init=0;
        }


        double Yellow[3] {0.5+0.3*sin(dt*M_PI/180), 0.502, 0.0};
        chessboard[box_n]->GetActor()->GetProperty()->SetColor(Yellow);
        dt=dt+20;

    }

    //-----------------POINTER: update position on the upper plane

    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    pointer_posit = tool_pose * KDL::Vector(-0.0, -0.0, -0.03 + 0.01);

    double x, y, z, w;
    tool_pose.M.GetQuaternion(x, y, z, w);

    KDL::Vector position;

    position=rot_inv*pointer_posit;
    position.x(position.x()+0.01);
    position.y(position.y()+0.038);

    double pointer_pose[7]= {
        pointer_posit[0], pointer_posit[1], pointer_posit[2], x, y, z, w};

    //position[2]<(height+kine_pointer_dim[0])

    if(position[2]<(height+kine_pointer_dim[0])) {

        position[0]=position[0]-0.01;
        position[1]=position[1]-0.038;
        position[2]=height+kine_pointer_dim[0];
        position=rot*position;
        pointer_pose[0]=position[0];
        pointer_pose[1]=position[1];
        pointer_pose[2]=position[2];

    }

    kine_p->SetKinematicPose(pointer_pose);

    PoseEvaluation();

    //-----------------

    //// map gripper value to an angle
    //double grip_posit =
    //    (*gripper_position[0]); //belonging to -0.5 - 1.55 interval
    //
    //if (grip_posit > threshold)
    //    cond = 1;
    //
    //if (grip_posit < threshold && cond == 1) {
    //    TaskEvaluation();
    //    cond = 0;
    //}

    //--------------------------------
    // step the world
    StepDynamicsWorld();
}

void TaskClutch::PoseEvaluation() {


    // Compute the distance of the target from the ideal value
    ideal_position = chessboard[box_n]->GetActor()->GetCenter();
    KDL::Vector id_pos;
    id_pos.x(ideal_position[0]);
    id_pos.y(ideal_position[1]);
    id_pos.z(ideal_position[2]);

    KDL::Vector id_position;
    id_position=rot_inv*id_pos;
    id_position.x(id_position.x()+0.01);
    id_position.y(id_position.y()+0.038);

    //std::cout<<id_position.x()<<std::endl;

    KDL::Vector position;
    position=rot_inv*pointer_posit;
    position.x(position.x()+0.01);
    position.y(position.y()+0.038);


    distance = id_position - position;

    KDL::Vector dist;
    dist.x( distance[0]);
    dist.y( distance[1]);
    dist.z( 0);

    double norm_distance = dist.Norm();


    if(norm_distance<3/2*kine_pointer_dim[0]) {


        if (task_state == TaskState::Idle){

            task_state = TaskState::Reaching;

        }

        if (task_state == TaskState::Reaching){

            chessboard[box_n]->GetActor()->GetProperty()->SetColor(color);
            box_n = rand() % (cols * rows) + 1;
            chessboard[box_n]->GetActor()->GetProperty()->GetColor(color);
            chessboard[box_n]->GetActor()->GetProperty()->SetColor(Green);
            task_rep=task_rep+1;
            if(task_rep>num_task_max) {
                task_state = TaskState::Idle;
                task_rep=0;

            }

        }

    }

}


//------------------------------------------------------------------------------
void TaskClutch::TaskEvaluation() {

    // Restore the color of the actual target to its original value
    chessboard[box_n]->GetActor()->GetProperty()->SetColor(color);

    // Compute the distance of the target from the ideal value
    ideal_position = chessboard[box_n]->GetActor()->GetCenter();
    KDL::Vector id_pos;
    id_pos.x(ideal_position[0]);
    id_pos.y(ideal_position[1]);
    id_pos.z(ideal_position[2]);
    distance = id_pos - pointer_posit;
    double norm_distance = distance.Norm();

    // Check if the target is inside the correct placement
    bool correct;
    if (id_pos.x() - pointer_posit.x() < board_dimensions[0] &&
        id_pos.y() - pointer_posit.y() < board_dimensions[1])
        correct = 1;
    else
        correct = 0;

    // Update new box target saving the actual color of the target and
    // changing the color of the new one
    box_n = rand() % (cols * rows) + 1;
    chessboard[box_n]->GetActor()->GetProperty()->GetColor(color);
    chessboard[box_n]->GetActor()->GetProperty()->SetColor(Green);

    // Computing Area of Correct Placement (ACP), i.e. the fraction of the area
    // of the pointer superimposed to the target

    // Need to find theta to define the circular sector
    double temp_x = board_dimensions[0] / 2 - fabs(distance.x());
    double temp_y = board_dimensions[1] / 2 - fabs(distance.y());

    if (temp_x > (-kine_pointer_dim[0])
        && temp_y > (-kine_pointer_dim[0])) {
        double chord_length =
            sqrt(
                pow(
                    sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_x, 2.0))
                        + temp_y, 2.0
                ) +
                    pow(
                        sqrt(
                            pow(kine_pointer_dim[0], 2.0)
                                - pow(temp_y, 2.0))
                            + temp_x, 2.0
                    ));
        double theta =
            2 * asin((chord_length / (2 * kine_pointer_dim[0])));
        std::cout << "theta: " << theta << std::endl;
        // THETA ALWAYS NAN, DON'T KNOW WHY

        // Composition of Surfaces
        double
            Asector = kine_pointer_dim[0] * kine_pointer_dim[0] * theta / 2;
        double Arectangle = temp_x * temp_y;
        std::cout << "A rect: " << Arectangle << std::endl;

        double Atriangle1 = temp_x *
            sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_x, 2.0)) / 2;
        std::cout << "A triang1: " << Atriangle1 << std::endl;

        double Atriangle2 = temp_y *
            sqrt(pow(kine_pointer_dim[0], 2.0) - pow(temp_y, 2.0)) / 2;
        std::cout << "A triang2: " << Atriangle2 << std::endl;

        double Aoverlap = Asector + Arectangle + Atriangle1 + Atriangle2;
        std::cout << "A overlap: " << Aoverlap << std::endl;

        // ACP
        ACP = (Aoverlap / (M_PI * pow(kine_pointer_dim[0], 2.0))) * 100;
    } else
        ACP = 0;

    // Cycle accessed only when the task begins (after an initial grip of
    // start).
    // It generates the message at each trial
    if (start == 1) {
        task_state_msg.task_name = "3D Test";
        time = (ros::Time::now() - begin).toSec();
        task_state_msg.time_stamp = time;
        task_state_msg.number_of_repetition = rep;
        rep = rep + uint8_t(1);
        task_state_msg.error_field_1 = norm_distance;
        task_state_msg.task_state = uint8_t(correct);
        task_state_msg.error_field_2 = ACP;
    }
    // Initialization
    start = 1;
    begin = ros::Time::now();

    std::cout << task_state_msg << std::endl;
}


//------------------------------------------------------------------------------
bool TaskClutch::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters TaskClutch::GetACParameters() {
    custom_msgs::ActiveConstraintParameters msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}

custom_msgs::TaskState TaskClutch::GetTaskStateMsg() {
    return task_state_msg;
}

void TaskClutch::ResetTask() {
    ROS_INFO("Repetition completed. Resetting the task.");

}

void TaskClutch::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}

void TaskClutch::FindAndPublishDesiredToolPose() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
                             ("/PSM1/tool_pose_desired", 10);
    if (bimanual)
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                                 ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok()) {

//        CalculatedDesiredToolPose();

        // publish desired poses
        for (int n_arm = 0; n_arm < 1 + int(bimanual); ++n_arm) {

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

void TaskClutch::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(
        dispatcher,
        overlappingPairCache, solver,
        collisionConfiguration
    );

    dynamicsWorld->setGravity(btVector3(0, 0, -10));

}

void TaskClutch::StepDynamicsWorld() {
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

TaskClutch::~TaskClutch() {

    ROS_INFO("Destructing Bullet task: %d",
             dynamicsWorld->getNumCollisionObjects());
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject
            *obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody *body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
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




