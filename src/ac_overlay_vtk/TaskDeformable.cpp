//
// Created by nima on 13/06/17.
//

#include "TaskDeformable.h"

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <vtkTriangle.h>




TaskDeformable::TaskDeformable(const std::string mesh_files_dir,
                       const bool show_ref_frames, const bool biman,
                       const bool with_guidance)
    :
    VTKTask(show_ref_frames, biman, with_guidance, 0),
    time_last(ros::Time::now())

{

    InitBullet();

    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject* floor= new BulletVTKObject(ObjectShape::STATICPLANE,
                                                ObjectType::DYNAMIC, floor_dims,
                                                dummy_pose, 0.0, 0, 0,
                                                NULL);
    dynamics_world->addRigidBody(floor->GetBody());


    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    //board_dimensions[0]  = 0.18;
    //board_dimensions[1]  = 0.14;
    //board_dimensions[2]  = 0.1;
    float attention_center[3] = {0.07, 0.07, 0.04};

    board_dimensions[0]  = 0.1;
    board_dimensions[1]  = 0.1;
    board_dimensions[2]  = 0.02;
    double friction = 0.1;

    double board_pose[7] = {attention_center[0] ,
        attention_center[1],
        -board_dimensions[2]/2,
        0, 0, 0, 1};

    std::vector<double> dim = { board_dimensions[0], board_dimensions[1],
        board_dimensions[2]};
    BulletVTKObject* board = new BulletVTKObject(ObjectShape::BOX,
                                                 ObjectType::DYNAMIC, dim,
                                                 board_pose, 0.0, 0, friction,
                                                 NULL);
    board->GetActor()->GetProperty()->SetOpacity(1.0);
    board->GetActor()->GetProperty()->SetColor(0.2, 0.3, 0.1);

    dynamics_world->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());



    // -------------------------------------------------------------------------
    // SOFT BODY
    float l = float(0.04 * B_DIM_SCALE);
    dynamics_world->setGravity(btVector3(0, 0, btScalar(-9.8)));

    sb_w_info = new btSoftBodyWorldInfo;
    sb_w_info->m_broadphase =overlappingPairCache;
    sb_w_info->m_dispatcher =dispatcher;
    sb_w_info->m_gravity=(btVector3(0, 0, btScalar(-9.8)));
    sb_w_info->m_sparsesdf.Initialize();

//    sb = btSoftBodyHelpers::CreatePatch(
//            *sb_w_info,
//            btVector3(0, 0, l/2),
//            btVector3(l, 0, l/2),
//            btVector3(0, l, l/2),
//            btVector3(l, l, l/2), 20, 20, 1+2+4+8, true);
//    sb->m_cfg.viterations = 50;
//    sb->setTotalMass(1);


    float density = 20000;
    float soft_pose[7] = {attention_center[0], attention_center[1],
                           attention_center[2]*2, 0.0, 0.0, 0.0, 1.0};
    std::stringstream input_file_dir;
    input_file_dir << mesh_files_dir << std::string("task_deformable_sphere.obj");
    std::string mesh_file_dir_str = input_file_dir.str();

    soft_o0 = new BulletVTKSoftObject(*sb_w_info, mesh_file_dir_str, soft_pose,
                                     density, friction);
    dynamics_world->addSoftBody(soft_o0->GetBody());
    actors.push_back(soft_o0->GetActor());

    soft_pose[0]+=0.03;
    soft_pose[2]+=0.05;
    soft_o1 = new BulletVTKSoftObject(*sb_w_info, mesh_file_dir_str, soft_pose,
                                     density, friction);
    dynamics_world->addSoftBody(soft_o1->GetBody());
    actors.push_back(soft_o1->GetActor());



    soft_pose[0]-=0.06;
    soft_pose[2]+=0.05;
    soft_o2 = new BulletVTKSoftObject(*sb_w_info, mesh_file_dir_str, soft_pose,
                                     density, friction);
    dynamics_world->addSoftBody(soft_o2->GetBody());
    actors.push_back(soft_o2->GetActor());



    //// -------------------------------------------------------------------------
    //// Create spheres
    //int cols = 3;
    //int rows = 2;
    //double density = 4; // kg/cm3
    //double * sphere_pose;
    //
    //BulletVTKObject* spheres[cols*rows];
    //for (int i = 0; i < rows; ++i) {
    //
    //    for (int j = 0; j < cols; ++j) {
    //
    //        std::vector<double> dim = {0.004};
    //
    //        sphere_pose = new double[7]{
    //            attention_center[0] + (double)i * 4*dim[0] + (double)j * dim[0]/2,
    //            attention_center[1],
    //            attention_center[2] + 0.12 + dim[0] *1.5* (double)j,
    //            0, 0, 0, 1};
    //
    //        spheres[i*rows+j] =
    //            new BulletVTKObject(
    //                ObjectShape::SPHERE, ObjectType::DYNAMIC, dim, sphere_pose,
    //                density, NULL, friction
    //            );
    //        delete [] sphere_pose;
    //        double ratio = (double)i/4.0;
    //        spheres[i*rows+j]->GetActor()->GetProperty()->SetColor(
    //            0.8 - 0.2*ratio, 0.4 - 0.3*ratio, 0.2 + 0.3*ratio);
    //        spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecular(0.8);
    //        spheres[i*rows+j]->GetActor()->GetProperty()->SetSpecularPower(50);
    //
    //        dynamics_world->addRigidBody(spheres[i*rows+j]->GetBody());
    //        actors.push_back(spheres[i*rows+j]->GetActor());
    //
    //    }
    //}

//    // -------------------------------------------------------------------------
//    // Create cubes
//    rows = 3;
//    cols = 2;
//    int layers = 3;
//    BulletVTKObject* cubes[layers *rows *cols];
//
//    double sides = 0.01;
//    density = 7000; // kg/m3
//    stiffnes = 1000;
//    damping = 5.1;
////    friction = 0.1;
//
//    for (int k = 0; k < layers; ++k) {
//        for (int i = 0; i < rows; ++i) {
//            for (int j = 0; j < cols; ++j) {
//
//                pose = new double[7] {(double)i * 2.2*sides + 0.1,
//                    (double)j * 2.2*sides  + 0.05,
//                    (double)k * 4*sides  + 0.01,
//                    0, 0, 0, 1};
//
//                std::vector<double> dim = {sides, sides, 2*sides};
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
////                dynamics_world->addRigidBody(cubes[i*rows+j]->GetBody());
////                actors.push_back(cubes[i*rows+j]->GetActor());
//
//            }
//        }
//    }

    {
//        std::vector<double> dim = {sides, sides, sides};
//        cubes[i*rows+j] = new BulletVTKObject(ObjectShape::BOX,
//                                              ObjectType::DYNAMIC, dim,
//                                              pose, 0.2), stiffnes, damping;

    }

    // -------------------------------------------------------------------------
    // Create mesh
//    stiffnes = 1000;
//    damping= 1;
////    friction = 1;
//
//    pose = new double[7] {0.06, 0.06, 0.1, 0.7, 0, 0.7, 0};
//    std::vector<double> _dim = {0.002};
//    BulletVTKObject *mesh;
//    std::stringstream input_file_dir;
//    input_file_dir << mesh_files_dir << std::string("monkey.obj");
//    std::string mesh_file_dir_str = input_file_dir.str();
//
//    mesh = new
//        BulletVTKObject(
//        ObjectShape::MESH, ObjectType::DYNAMIC, _dim, pose, 6000,
//        &mesh_file_dir_str, friction
//    );
//
////    dynamics_world->addRigidBody(mesh->GetBody());
////    actors.push_back(mesh->GetActor());
//    mesh->GetActor()->GetProperty()->SetColor(0., 0.9, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic box

    std::vector<double> kine_box_dim = {0.002, 0.002, 0.01};
    //kine_box =
    //    new BulletVTKObject(
    //        ObjectShape::BOX, ObjectType::KINEMATIC, kine_box_dim, dummy_pose, 0.0,
    //        NULL, friction
    //    );
    //dynamics_world->addRigidBody(kine_box->GetBody());
    //actors.push_back(kine_box->GetActor());
    //kine_box->GetActor()->GetProperty()->SetColor(1., 0.1, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic sphere

    std::vector<double> kine_sph_dim = {0.002};
    kine_sphere_0 =
            new BulletVTKObject(ObjectShape::SPHERE, ObjectType::KINEMATIC,
                                kine_sph_dim, dummy_pose, 0.0, 0, friction,
                                NULL);
    dynamics_world->addRigidBody(kine_sphere_0->GetBody());
    actors.push_back(kine_sphere_0->GetActor());
    kine_sphere_0->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic sphere

    kine_sphere_1 =
            new BulletVTKObject(ObjectShape::SPHERE, ObjectType::KINEMATIC,
                                kine_sph_dim, dummy_pose, 0.0, 0, friction,
                                NULL);

    dynamics_world->addRigidBody(kine_sphere_1->GetBody());
    actors.push_back(kine_sphere_1->GetActor());
    kine_sphere_1->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);

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
void TaskDeformable::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}


void TaskDeformable::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};

//------------------------------------------------------------------------------
void TaskDeformable::UpdateActors() {

    soft_o0->RenderSoftbody();
    soft_o1->RenderSoftbody();
    soft_o2->RenderSoftbody();
    //--------------------------------
    //box
    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    KDL::Vector box_posit = tool_pose * KDL::Vector( 0.0 , 0.0, -0.03);

    //double x, y, z, w;
    //tool_pose.M.GetQuaternion(x,y,z,w);
    //double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2],x,y,z,w};
    //kine_box->SetKinematicPose(box_pose);


    //--------------------------------
    //sphere 0
    double grip_posit = (*gripper_position[0]);

    KDL::Vector gripper_pos = KDL::Vector( 0.0, (1+grip_posit)* 0.002, 0.001);
    gripper_pos = tool_pose * gripper_pos;

    double sphere_0_pose[7] = {
        gripper_pos[0],
        gripper_pos[1],
        gripper_pos[2],
        0,0,0,1};
    kine_sphere_0->SetKinematicPose(sphere_0_pose);


    //--------------------------------
    //sphere 1
    gripper_pos = KDL::Vector( 0.0, -(1+grip_posit)* 0.002, 0.001);
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
bool TaskDeformable::IsACParamChanged() {
    return false;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters * TaskDeformable::GetACParameters() {
    custom_msgs::ActiveConstraintParameters *msg;
    // assuming once we read it we can consider it unchanged
    return msg;
}


custom_msgs::TaskState TaskDeformable::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskDeformable::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskDeformable::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskDeformable::FindAndPublishDesiredToolPose() {

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





void TaskDeformable::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    sb_solver = new btDefaultSoftBodySolver;

    dynamics_world = new btSoftRigidDynamicsWorld(dispatcher,
                                                  overlappingPairCache, solver,
                                                  collisionConfiguration, sb_solver);

    dynamics_world->setGravity(btVector3(0, 0, btScalar(-9.8)));


}


void TaskDeformable::StepDynamicsWorld() {
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


TaskDeformable::~TaskDeformable() {

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

    delete sb_solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;


    //next line is optional: it will be cleared by the destructor when the array goes out of scope
//    collisionShapes.clear();
}

void TaskDeformable::RenderSoftbody(
    btSoftBody *b,
    vtkSmartPointer<vtkActor> actor
) {
    // first attempt. I doubt it is the most efficient way.
    //faces
    vtkSmartPointer<vtkCellArray> triangles =
        vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    // Create a polydata object
    vtkSmartPointer<vtkPolyData> polyData =
        vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    std::vector<vtkSmartPointer<vtkTriangle> >triangle;

    for(int i=0;i<b->m_faces.size();i++)
    {
        triangle.push_back(vtkSmartPointer<vtkTriangle>::New());

        for(int j=0;j<3;j++) {
            points->InsertNextPoint(b->m_faces[i].m_n[j]->m_x.x()/B_DIM_SCALE,
                                    b->m_faces[i].m_n[j]->m_x.y()/B_DIM_SCALE,
                                    b->m_faces[i].m_n[j]->m_x.z()/B_DIM_SCALE);

            triangle[i]->GetPointIds()->SetId ( j, i*3  + j );
        }

        triangles->InsertNextCell ( triangle[i] );
    }
    // Add the geometry and topology to the polydata
    polyData->SetPoints ( points );
    polyData->SetPolys ( triangles );
    mapper->SetInputData(polyData);
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.1, 0.6, 0.7);
////    for(int i=0;i<b->m_links.size();i++)
//    {
//        for(int j=0;j<2;j++)
//            (b->m_links[i].m_n[j]->m_x.x(),
//                       b->m_links[i].m_n[j]->m_x.y(),
//                       b->m_links[i].m_n[j]->m_x.z());
//    }

}


