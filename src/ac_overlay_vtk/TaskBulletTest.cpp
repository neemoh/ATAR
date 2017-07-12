//
// Created by nima on 13/06/17.
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



    BulletVTKObject* board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    //board_dimensions[0]  = 0.18;
    //board_dimensions[1]  = 0.14;
    //board_dimensions[2]  = 0.1;

    board_dimensions[0]  = 0.48;
    board_dimensions[1]  = 0.54;
    board_dimensions[2]  = 0.05;
    double *pose;
    double density;
    double stiffnes = 1000;
    double damping = 20;
    double friction = 6;

    pose= new double[7] {board_dimensions[0]/2,
        board_dimensions[1] / 2,
        0,
        0, 0, 0, 1};

    std::vector<double> dim = { board_dimensions[0], board_dimensions[1],
        board_dimensions[2]};
    board = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose, 0.0, NULL, friction
    );
    board->GetActor()->GetProperty()->SetOpacity(0.0);
    board->GetActor()->GetProperty()->SetColor(0.8, 0.3, 0.1);

    dynamicsWorld->addRigidBody(board->GetBody());
    actors.push_back(board->GetActor());


    // -------------------------------------------------------------------------
    //// Create spheres
    //int cols = 4;
    //int rows = 3;
    //double density = 7000; // kg/m3
    //stiffnes = 1000;
    //damping = 0.1;
    //friction = 0.2;
    //BulletVTKObject* cylinders[cols*rows];
    //for (int i = 0; i < rows; ++i) {
    //
    //    for (int j = 0; j < cols; ++j) {
    //
    //        std::vector<double> dim = {0.005, 0.05};
    //
    //        pose = new double[7]{(double)i * 4*dim[0] + (double)j * dim[0]/2,
    //            0.06,
    //            0.08 + dim[1] *1.5* (double)j,
    //            0, 0, 0, 1};
    //
    //        cylinders[i*rows+j] =
    //            new BulletVTKObject(ObjectShape::CYLINDER,
    //                                ObjectType::DYNAMIC, dim, pose, density,
    //                                NULL, friction, stiffnes, damping
    //            );
    //        delete [] pose;
    //        double ratio = (double)i/4.0;
    //        cylinders[i*rows+j]->GetActor()->GetProperty()->SetColor(
    //            0.6 - 0.2*ratio, 0.6 - 0.3*ratio, 0.7 + 0.3*ratio);
    //        cylinders[i*rows+j]->GetActor()->GetProperty()->SetSpecular(0.8);
    //        cylinders[i*rows+j]->GetActor()->GetProperty()->SetSpecularPower(50);
    //
    //        dynamicsWorld->addRigidBody(cylinders[i*rows+j]->GetBody());
    //        actors.push_back(cylinders[i*rows+j]->GetActor());
    //
    //    }
    //}

    // -------------------------------------------------------------------------
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
//    friction = 0.1;
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
//                cubes[i*rows+j] = new BulletVTKObject(ObjectShape::BOX,
//                                                      ObjectType::DYNAMIC, dim,
//                                                      pose, density, NULL,
//                                                      friction, stiffnes, damping);
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

//    {
//        std::vector<double> dim = {sides, sides, sides};
//        cubes[i*rows+j] = new BulletVTKObject(ObjectShape::BOX,
//                                              ObjectType::DYNAMIC, dim,
//                                              pose, 0.2), stiffnes, damping;
//
//    }

    // -------------------------------------------------------------------------
    //// Create mesh
    //stiffnes = 1000;
    //damping= 1;
    //friction = 1;
    //
    //pose = new double[7] {0.06, 0.06, 0.1, 0.7, 0, 0.7, 0};
    //std::vector<double> _dim = {0.002};
    //BulletVTKObject *mesh;
    //std::stringstream input_file_dir;
    //input_file_dir << mesh_files_dir << std::string("monkey.obj");
    //std::string mesh_file_dir_str = input_file_dir.str();
    //
    //mesh = new
    //    BulletVTKObject(ObjectShape::MESH,
    //                    ObjectType::DYNAMIC, _dim, pose, 6000,
    //                    &mesh_file_dir_str,
    //                    friction);
    //dynamicsWorld->addRigidBody(mesh->GetBody());
    //actors.push_back(mesh->GetActor());
    //mesh->GetActor()->GetProperty()->SetColor(0., 0.9, 0.1);

    // -------------------------------------------------------------------------
    // Create kinematic box

    stiffnes = 1000;
    damping= 100;
    friction = 50.1;


    pose = new double[7] {0, 0, 0, 0, 0, 0, 1};
    std::vector<double> kine_box_dim = {0.005, 0.005, 0.02};
    kine_box =
        new BulletVTKObject(
            ObjectShape::BOX, ObjectType::KINEMATIC, kine_box_dim, pose, 0.0,
            NULL, friction
        );
    dynamicsWorld->addRigidBody(kine_box->GetBody());
    kine_box->GetActor()->GetProperty()->SetColor(1., 0.1, 0.1);
    actors.push_back(kine_box->GetActor());


    // -------------------------------------------------------------------------
    // Create kinematic scoop

    std::vector<double> kine_scoop_dim = {0.02, 0.0002, 0.02};
    kine_scoop =
        new BulletVTKObject(
            ObjectShape::BOX, ObjectType::KINEMATIC, kine_scoop_dim, pose, 10.0,
            NULL, friction
        );
    dynamicsWorld->addRigidBody(kine_scoop->GetBody());
    actors.push_back(kine_scoop->GetActor());
    kine_scoop->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);

    // -------------------------------------------------------------------------
    //// Create kinematic cylinder
    //     std::vector<double> kine_cyl_dim = {0.01, 0.0002};
    //kine_cylinder_1 =
    //    new BulletVTKObject(ObjectShape::CYLINDER,
    //                        ObjectType::KINEMATIC, kine_cyl_dim, pose, 10.0,
    //                        NULL, friction, stiffnes, damping);
    //delete [] pose;
    //dynamicsWorld->addRigidBody(kine_cylinder_1->GetBody());
    //actors.push_back(kine_cylinder_1->GetActor());
    //kine_cylinder_1->GetActor()->GetProperty()->SetColor(1., 0.4, 0.1);


    // -------------------------------------------------------------------------
    // Create pegs and spheres

    peg_dimensions[0]  = 0.008;
    peg_dimensions[1]  = 0.008;
    peg_dimensions[2]  = 0.008;

    // Set cubic pegs params
    density = 12;
    stiffnes = 1000;
    damping = 20;
    friction = 100;

    // Set spheric pegs params
    double stiffnes_sphere = 10;
    double damping_sphere = 10;
    double friction_sphere = 0.2;

    if (peg_type==1){
        // -------------------------------------------------------------------------
        // Create sphere 1

        peg_pose1 = new double[7]{
            0.04, 0.04, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        std::vector<double> peg_SPHERE_dimension = {peg_dimensions[0] / 2};

        peg1 = new BulletVTKObject(
            ObjectShape::SPHERE, ObjectType::DYNAMIC, peg_SPHERE_dimension,
            peg_pose1, density, NULL, friction_sphere
        );
        peg1->GetActor()->GetProperty()->SetOpacity(1.0);
        peg1->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg1->GetBody());
        actors.push_back(peg1->GetActor());


        // -------------------------------------------------------------------------
        // Create sphere 2

        peg_pose2 = new double[7]{
            0.08, 0.04, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };


        peg2 = new BulletVTKObject(
            ObjectShape::SPHERE, ObjectType::DYNAMIC, peg_SPHERE_dimension,
            peg_pose2, density, NULL, friction_sphere
        );
        peg2->GetActor()->GetProperty()->SetOpacity(1.0);
        peg2->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg2->GetBody());
        actors.push_back(peg2->GetActor());

        // -------------------------------------------------------------------------
        // Create sphere 3

        peg_pose3 = new double[7]{
            0.04, 0.08, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };


        peg3 = new BulletVTKObject(
            ObjectShape::SPHERE, ObjectType::DYNAMIC, peg_SPHERE_dimension,
            peg_pose3, density, NULL, friction_sphere
        );
        peg3->GetActor()->GetProperty()->SetOpacity(1.0);
        peg3->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg3->GetBody());
        actors.push_back(peg3->GetActor());

        // -------------------------------------------------------------------------
        // Create sphere 4

        peg_pose4 = new double[7]{
            0.06, 0.06, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        peg4 = new BulletVTKObject(
            ObjectShape::SPHERE, ObjectType::DYNAMIC, peg_SPHERE_dimension,
            peg_pose4, density, NULL, friction_sphere
        );
        peg4->GetActor()->GetProperty()->SetOpacity(1.0);
        peg4->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg4->GetBody());
        actors.push_back(peg4->GetActor());

    }

    if (peg_type==0) {
        // -------------------------------------------------------------------------
        // Create  cubic peg1

        peg_pose1 = new double[7]{
            0.0, 0.0, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        std::vector<double> peg_dim = {
            peg_dimensions[0], peg_dimensions[1], peg_dimensions[2]
        };
        peg1 = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, peg_dim, peg_pose1, density,
            NULL, friction
        );
        peg1->GetActor()->GetProperty()->SetOpacity(1.0);
        peg1->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg1->GetBody());
        actors.push_back(peg1->GetActor());


        // -------------------------------------------------------------------------
        // Create cubic peg2

        peg_pose2 = new double[7]{
            0.04, 0.0, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        peg2 = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, peg_dim, peg_pose2, density,
            NULL, friction
        );
        peg2->GetActor()->GetProperty()->SetOpacity(1.0);
        peg2->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg2->GetBody());
        actors.push_back(peg2->GetActor());

        // -------------------------------------------------------------------------
        // Create cubic peg3

        peg_pose3 = new double[7]{
            0.0, 0.04, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        peg3 = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, peg_dim, peg_pose3, density,
            NULL, friction
        );
        peg3->GetActor()->GetProperty()->SetOpacity(1.0);
        peg3->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg3->GetBody());
        actors.push_back(peg3->GetActor());

        // -------------------------------------------------------------------------
        // Create cubic peg4

        peg_pose4 = new double[7]{
            0.02, 0.02, board_dimensions[2]/2+peg_dimensions[2]/2, 0, 0, 0, 1
        };

        peg4 = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, peg_dim, peg_pose4, density,
            NULL, friction
        );
        peg4->GetActor()->GetProperty()->SetOpacity(1.0);
        peg4->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

        dynamicsWorld->addRigidBody(peg4->GetBody());
        actors.push_back(peg4->GetActor());
    }

    //// Create a ring (mesh object)
    //
    //density=10;
    //stiffnes = 100;
    //damping= 20;
    //friction = 10;
    //
    //pose = new double[7] {0.05, 0.01,  0,
    //    0, 0, 0, 1};
    //std::vector<double> _dim = {0.002};
    //BulletVTKObject *mesh;
    //std::stringstream input_file_dir;
    //
    ////to change
    //input_file_dir << mesh_files_dir << std::string("ring.obj");
    //
    //
    //std::string mesh_file_dir_str = input_file_dir.str();
    //
    //mesh = new
    //    BulletVTKObject(ObjectShape::MESH,
    //                    ObjectType::DYNAMIC, _dim, pose, density,
    //                    &mesh_file_dir_str,
    //                    friction, stiffnes, damping);
    //
    //dynamicsWorld->addRigidBody(mesh->GetBody());
    //actors.push_back(mesh->GetActor());
    //mesh->GetActor()->GetProperty()->SetColor(0., 0.9, 0.1);


    // -------------------------------------------------------------------------
    // Create a static target made of cubes

    sides = peg_dimensions[0]+0.002;
    stiffnes = 1000;
    damping = 100;
    friction = 0.51;
    std::vector<double> dim1 = {sides/2, 3*sides-0.5*sides, 2*sides};
    std::vector<double> dim2 = {2*sides-0.5*sides, sides/2, 2*sides};

    target_pos = {0.12, 0.11, board_dimensions[2]-0.01};

    // Create Cube1

    pose = new double[7] {target_pos[0]+sides, target_pos[1], target_pos[2],
        0, 0, 0, 1};

    cubes[0] = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim1, pose, 0.0, NULL, friction
    );
    cubes[0]->GetActor()->GetProperty()->SetColor(
        0.9, 0.4, 0.1);
    dynamicsWorld->addRigidBody(cubes[0]->GetBody());
    actors.push_back(cubes[0]->GetActor());

    // Create Cube2

    pose = new double[7] {target_pos[0], target_pos[1]-sides, target_pos[2],
        0, 0, 0, 1};

    cubes[1] = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim2, pose, 0.0, NULL, friction
    );
    cubes[1]->GetActor()->GetProperty()->SetColor(
        0.9, 0.4, 0.1);
    dynamicsWorld->addRigidBody(cubes[1]->GetBody());
    actors.push_back(cubes[1]->GetActor());

    // Create Cube3

    pose = new double[7] {target_pos[0], target_pos[1]+sides, target_pos[2],
        0, 0, 0, 1};

    cubes[2] = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim2, pose, 0.0, NULL, friction
    );
    cubes[2]->GetActor()->GetProperty()->SetColor(
        0.9, 0.4, 0.1);
    dynamicsWorld->addRigidBody(cubes[2]->GetBody());
    actors.push_back(cubes[2]->GetActor());

    // Create Cube4

    pose = new double[7] {target_pos[0]-sides, target_pos[1], target_pos[2],
        0, 0, 0, 1};

    cubes[3] = new BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim1, pose, 0.0, NULL, friction
    );
    cubes[3]->GetActor()->GetProperty()->SetColor(
        0.9, 0.4, 0.1);
    dynamicsWorld->addRigidBody(cubes[3]->GetBody());
    actors.push_back(cubes[3]->GetActor());

    // -------------------------------------------------------------------------
    // FRAMES
    //vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
    //    vtkSmartPointer<vtkAxesActor>::New();
    //
    //task_coordinate_axes->SetXAxisLabelText("");
    //task_coordinate_axes->SetYAxisLabelText("");
    //task_coordinate_axes->SetZAxisLabelText("");
    //task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    //task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    //
    //
    //actors.push_back(task_coordinate_axes);

    // compute the euclidean distance between the peg and the target
    // (this should be the ideal path)
    //KDL::Vector peg_position(peg_pose[0], peg_pose[1], peg_pose[2]);
    //KDL::Vector target_position(target_pos[0], target_pos[1], target_pos[2]);
    //target_distance=(target_position - peg_position).Norm();
    //previous_point = peg_position;
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

    //--------------------------------
    //box
    KDL::Frame tool_pose = (*tool_current_pose_kdl[0]);

    KDL::Vector box_posit = tool_pose * KDL::Vector( -0.0, -0.0, -0.03+0.01);

    double x, y, z, w;
    tool_pose.M.GetQuaternion(x,y,z,w);
    double box_pose[7] = {box_posit[0], box_posit[1], box_posit[2],x,y,z,w};
    kine_box->SetKinematicPose(box_pose);


    //--------------------------------
    //scoop
    double grip_posit = (*gripper_position[0]);

    KDL::Vector gripper_pos = KDL::Vector( -0.0, -0.0, -0.01+0.01); //previously (4 the cylinder) y=(1+grip_posit)* 0.004
    gripper_pos = tool_pose * gripper_pos;

    double scoop_pose[7] = {
        gripper_pos[0],
        gripper_pos[1],
        gripper_pos[2],
        x,y,z,w};
    kine_scoop->SetKinematicPose(scoop_pose);

    //--------------------------------
    // step the world
    StepDynamicsWorld();

    //--------------------------------
    // Check if the task is completed
    EndChecking();

}


//------------------------------------------------------------------------------
void TaskBulletTest::EndChecking(){
    // Check if the pegs are in the correct position.
        double* peg_position;
        peg_position = peg1->GetActor()->GetCenter();
        peg1->GetActor()->GetProperty()->SetColor(0.0, 0.9, 0.0);


    // check if the first peg is inside the target or fallen
        if (peg_position[2]<0){
            out[0]=1;

        }
        if ((fabs(peg_position[0]-target_pos[0])<=3*sides/2 && fabs(peg_position[1]-target_pos[1])<=3*sides/2) || out[0]==1) {

            // the first peg is in the target! the second peg turns green
            peg2->GetActor()->GetProperty()->SetColor(0.0, 0.9, 0.0);

            // check if the second peg is inside the target or fallen
            peg_position = peg2->GetActor()->GetCenter();

            if (peg_position[2]<0){
                out[1]=1;
            }
            if ((fabs(peg_position[0]-target_pos[0])<=3*sides/2 && fabs(peg_position[1]-target_pos[1])<=3*sides/2 && fabs(peg_position[2]==target_pos[2])<=3*sides) || out[1]==1) {

                // the second peg is in the target! the third peg turns green
                peg3->GetActor()->GetProperty()->SetColor(0.0, 0.9, 0.0);

                // check if the third peg is inside the target or fallen
                peg_position = peg3->GetActor()->GetCenter();

                if (peg_position[2]<0){
                    out[2]=1;
                }
                if ((fabs(peg_position[0]-target_pos[0])<=3*sides/2 && fabs(peg_position[1]-target_pos[1])<=3*sides/2 && fabs(peg_position[2]==target_pos[2])<=3*sides) || out[2]==1) {

                    // the third peg is in the target! the last peg turns green
                    peg4->GetActor()->GetProperty()->SetColor(0.0, 0.9, 0.0);

                    // check if the last peg is inside the target or fallen
                    peg_position = peg4->GetActor()->GetCenter();

                    if (peg_position[2]<0){ out[3]=1;
                }
                if ((fabs(peg_position[0]-target_pos[0])<=3*sides/2 && fabs(peg_position[1]-target_pos[1])<=3*sides/2 && fabs(peg_position[2]==target_pos[2])<=3*sides) || out[3]==1) {
                    std::cout << "daje regaz final" << std::endl;
                    ResetTask();
                }
            }
        }
    }
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
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskBulletTest::ResetTask() {

    // save the measurements and the metrics and reset the initial conditions
    // to start a new repetition of the task
    ROS_INFO("Repetition completed. Resetting the task.");

    // save metrics
    // DO WE SAVE THE METRICS IN A PARAMETER?
    // OR DO WE SAVE THEM THROUGH THE NODE REPORTER?

    //// reset
    for (int i = 0; i < 4; ++i) {
        out[i]=0;
    }

    peg1->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(peg_pose1));
    motion_state_ = new BulletVTKMotionState(peg_pose1, peg1->GetActor());
    peg1->GetBody()->setMotionState(motion_state_);
    peg1->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

    peg2->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(peg_pose2));
    motion_state_ = new BulletVTKMotionState(peg_pose2, peg2->GetActor());
    peg2->GetBody()->setMotionState(motion_state_);
    peg2->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

    peg3->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(peg_pose3));
    motion_state_ = new BulletVTKMotionState(peg_pose3, peg3->GetActor());
    peg3->GetBody()->setMotionState(motion_state_);
    peg3->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

    peg4->GetActor()->SetUserMatrix(PoseVectorToVTKMatrix(peg_pose4));
    motion_state_ = new BulletVTKMotionState(peg_pose4, peg4->GetActor());
    peg4->GetBody()->setMotionState(motion_state_);
    peg4->GetActor()->GetProperty()->SetColor(0.1, 0.2, 0.6);

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


