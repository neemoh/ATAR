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



    // -------------------------------------------------------------------------
    //  ACTIVE CONSTRAINT
    // -------------------------------------------------------------------------
    // these parameters could be set as ros parameters too but since
    // they change during the task I am hard coding them here.
    ac_parameters.method = 0; // 0 for visco/elastic
    ac_parameters.active = 0;

    ac_parameters.max_force = 4.0;
    ac_parameters.linear_elastic_coeff = 1000.0;
    ac_parameters.linear_damping_coeff = 10.0;

    ac_parameters.max_torque = 0.03;
    ac_parameters.angular_elastic_coeff = 0.04;
    ac_parameters.angular_damping_coeff = 0.002;

    d_board_actor = vtkSmartPointer<vtkActor>::New();




    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);



    // -------------------------------------------------------------------------
    // Create a cube for the board
    vtkSmartPointer<vtkCubeSource> board_source =
            vtkSmartPointer<vtkCubeSource>::New();
    board_dimensions[0]  = 0.18;
    board_dimensions[1]  = 0.14;
    board_dimensions[2]  = 0.01;

    board_source->SetXLength(board_dimensions[0]);
    board_source->SetYLength(board_dimensions[1]);
    board_source->SetZLength(board_dimensions[2]);
    vtkSmartPointer<vtkPolyDataMapper> board_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    board_mapper->SetInputConnection(board_source->GetOutputPort());
    d_board_actor->SetMapper(board_mapper);
    d_board_actor->SetPosition(board_dimensions[0] / 2,
                               board_dimensions[1] / 2,
                               -board_dimensions[2]);
    d_board_actor->GetProperty()->SetOpacity(1.00);
    double colr[3] {1.0, 1.0, 1.0};
    d_board_actor->GetProperty()->SetColor(colr);




    // -------------------------------------------------------------------------
    // Error history spheres

    vtkSmartPointer<vtkSphereSource>  source =
            vtkSmartPointer<vtkSphereSource>::New();

    source->SetRadius(RAD_SPHERES);
    source->SetPhiResolution(30);
    source->SetThetaResolution(30);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(source->GetOutputPort());

    vtkSmartPointer<vtkMinimalStandardRandomSequence> sequence =
            vtkSmartPointer<vtkMinimalStandardRandomSequence>::New();
    // initialize the sequence
    sequence->SetSeed(1);

    for (int i = 0; i < NUM_BULLET_SPHERES; ++i) {

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        sequence->GetRangeValue(0.0,1.0);
        double a = sequence->GetRangeValue(0.0,1.0);
        actor->GetProperty()->SetColor(0.6 - 0.2*a, 0.6 - 0.3*a, 0.7 + 0.3*a);
        sequence->Next();

        a = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double b = sequence->GetRangeValue(0.0, 0.1);
        sequence->Next();
        double c = sequence->GetRangeValue(0.0, 0.05);
        sequence->Next();
        sphere_positions.push_back({a,b, 0.07 + c});
        actor->SetPosition(sphere_positions[i][0],
                           sphere_positions[i][1],
                           sphere_positions[i][2]);
        actor->GetProperty()->SetSpecular(0.8);
        actor->GetProperty()->SetSpecularPower(50);
        d_sphere_actors.push_back(actor);
    }


    // -------------------------------------------------------------------------
    double source_scales = 0.006;


    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);
    }
    for (int j = 0; j < d_sphere_actors.size(); ++j) {
        actors.push_back(d_sphere_actors[j]);
    }

    actors.push_back(d_board_actor);


    InitBullet();

}


//------------------------------------------------------------------------------
void BulletTask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

//------------------------------------------------------------------------------
void BulletTask::UpdateActors() {

//    for (int i = 0; i < 5; ++i) {
    SimLoopODE();
//    }


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
//    number_of_repetition = 0;
//    task_state = BulletTaskState::RepetitionComplete;
//    ResetOnGoingEvaluation();
//    ResetScoreHistory();
}

void BulletTask::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
//    if(task_state== BulletTaskState::ToEndPoint ||
//       task_state == BulletTaskState::ToStartPoint){
//
//        ResetOnGoingEvaluation();
//        if(number_of_repetition>0)
//            number_of_repetition--;
//        task_state = BulletTaskState::RepetitionComplete;
//
//    }
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
    int i;
    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, 0, -10));

    ///-----initialization_end-----



    ///create a few basic rigid bodies

    //the ground is a cube of side 100 at position y = -56.
    //the sphere will hit it at y = -6, with center at -5
    {
        btCollisionShape* groundShape = new btBoxShape(
                btVector3(btScalar(board_dimensions[0])
                        , btScalar(board_dimensions[1]),
                          btScalar(board_dimensions[2])));

        collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3((float) (board_dimensions[0] / 2.45),
                                            (float) (board_dimensions[1] / 2.78),
                                            (float) -board_dimensions[2]/2));

        btScalar mass(0.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        dynamicsWorld->addRigidBody(body);
    }

    {
        //create a dynamic rigidbody
        for (int j = 0; j < NUM_BULLET_SPHERES; ++j) {


            //btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
            btCollisionShape* colShape = new btSphereShape(btScalar(RAD_SPHERES));
            collisionShapes.push_back(colShape);

            /// Create Dynamic Objects
            btTransform startTransform;
            startTransform.setIdentity();

            btScalar mass(1.f);

            //rigidbody is dynamic if and only if mass is non zero, otherwise static
            bool isDynamic = (mass != 0.f);

            btVector3 localInertia(0, 0, 0);
            if (isDynamic)
                colShape->calculateLocalInertia(mass, localInertia);

            startTransform.setOrigin(btVector3((float) sphere_positions[j][0],
                                               (float) sphere_positions[j][1],
                                               (float) sphere_positions[j][2]));

            //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
            btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
            btRigidBody* body = new btRigidBody(rbInfo);

            dynamicsWorld->addRigidBody(body);
        }
    }


}


void BulletTask::SimLoopODE() {
    ///-----stepsimulation_start-----

    dynamicsWorld->stepSimulation(1.f / 60.f, 10);

    //print positions of all objects
    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
        btRigidBody* body = btRigidBody::upcast(obj);
        btTransform trans;
        if (body && body->getMotionState())
        {
            body->getMotionState()->getWorldTransform(trans);
        }
        else
        {
            trans = obj->getWorldTransform();
        }
//        printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));

        if (j==0)
            d_board_actor->SetPosition(trans.getOrigin().getX(),
                                       trans.getOrigin().getY(),
                                       trans.getOrigin().getZ());
        else
            d_sphere_actors[j-1]->SetPosition(trans.getOrigin().getX(),
                                              trans.getOrigin().getY(),
                                              trans.getOrigin().getZ());
    }


}


BulletTask::~BulletTask() {
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

    //delete collision shapes
    for (int j = 0; j < collisionShapes.size(); j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = 0;
        delete shape;
    }

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
    collisionShapes.clear();
}


