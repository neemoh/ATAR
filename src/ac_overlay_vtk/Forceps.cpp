//
// Created by nima on 10/08/17.
//

#include "Forceps.h"
#include <vtkProperty.h>
#include <sstream>

Forceps::Forceps(
    const std::string mesh_dir
)
        : num_links_(3)
{

    link_dims_.push_back({0.003, 0.003, 0.003});
    float gripper_density = 500000; // kg/m3
    float gripper_friction = 50;
    {
        double gripper_pose[7]{0.09, 0.15, 0.09, 0, 0, 0, 1};


        gripper_links[0] =
                new BulletVTKObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                    link_dims_[0], gripper_pose,
                                    gripper_density, 0);
        gripper_links[0]->GetActor()->GetProperty()->SetColor(0.65f, 0.7f,
                                                              0.7f);
        gripper_links[0]->GetBody()->setContactStiffnessAndDamping(2000, 100);
    }

    std::stringstream input_file_dir;
    input_file_dir << mesh_dir
                   << std::string("jaw.obj");
    std::string mesh_file_dir_str = input_file_dir.str();
    {
        double gripper_pose[7]{0.09, 0.15, 0.09, 0, 0, 1, 0};


        gripper_links[1] =
                new BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                                    link_dims_[0], gripper_pose,
                                    gripper_density, 1, gripper_friction,
                                    &mesh_file_dir_str);
        gripper_links[1]->GetActor()->GetProperty()->SetColor(0.f, 0.7f,
                                                              0.7f);
        gripper_links[1]->GetBody()->setContactStiffnessAndDamping(2000, 100);
    }

    {
        double gripper_pose[7]{0.09, 0.15, 0.09, 0, 0, 0, 1};

        gripper_links[2] =
            new BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                                link_dims_[0], gripper_pose,
                                gripper_density, 1, gripper_friction,
                                &mesh_file_dir_str);
        gripper_links[2]->GetActor()->GetProperty()->SetColor(0.65f, 0.7f,
                                                              0.7f);
        gripper_links[2]->GetBody()->setContactStiffnessAndDamping(1000, 200);
    }

    auto jaws_axis_offset = -0.001 * B_DIM_SCALE;
    const btVector3 pivot_link0(0.f, 0.f,  0.008f  * B_DIM_SCALE);
    const btVector3 pivot_jaw_1(0.f, jaws_axis_offset, 0.f);
    const btVector3 pivot_jaw_2(0.f, jaws_axis_offset, 0.f);

    btVector3 btAxisA( 1.0f, 0.f, 0.0f );
    btVector3 btAxisB( 0.0f, -1.f, 0.f );
    btVector3 btAxisC( 0.0f, 1.f, 0.f );

    hinges[0] = new btHingeConstraint(
            *gripper_links[0]->GetBody(),
            *gripper_links[1]->GetBody(),
            pivot_link0, pivot_jaw_1, btAxisA, btAxisB );
    hinges[0]->enableMotor(true);
    hinges[0]->setMaxMotorImpulse(200);
//    hinges[0]->setLimit(0, M_PI/4);

    hinges[1] = new btHingeConstraint(
            *gripper_links[0]->GetBody(),
            *gripper_links[2]->GetBody(),
            pivot_link0, pivot_jaw_2, btAxisA, btAxisC );
    hinges[1]->enableMotor(true);
    hinges[1]->setMaxMotorImpulse(200);
}

void Forceps::SetPoseAndJawAngle(const KDL::Frame pose,
                                          const double grip_angle) {

    KDL::Frame grpr_links_pose[5];

    //-------------------------------- LINK 0
    grpr_links_pose[0] = pose;
    grpr_links_pose[0].p  = grpr_links_pose[0] * KDL::Vector( 0.0 , 0.0,
                                                              -link_dims_[0][2]/2);
    double x, y, z, w;
    pose.M.GetQuaternion(x,y,z,w);
    double link0_pose[7] = {grpr_links_pose[0].p.x(),
                            grpr_links_pose[0].p.y(), grpr_links_pose[0].p.z(),x,y,z,w};
    gripper_links[0]->SetKinematicPose(link0_pose);

    //-------------------------------- LINK 1
    //grpr_links_pose[1] = pose;
    //grpr_links_pose[1].M.DoRotX(-grip_angle);
    //grpr_links_pose[1].p =  grpr_links_pose[1] *
    //                        KDL::Vector( 0.0, 0.0, link_dims_[1][2]/2);
    //grpr_links_pose[1].M.GetQuaternion(x, y, z, w);
    //
    //double link2_pose[7] = {grpr_links_pose[1].p.x(),
    //                        grpr_links_pose[1].p.y(), grpr_links_pose[1].p.z(), x, y, z, w};

//    gripper_links[1]->SetKinematicPose(link2_pose);

    hinges[0]->setMotorTarget(grip_angle, 0.05);
    //-------------------------------- LINK 2
    //grpr_links_pose[2] = pose;
    //grpr_links_pose[2].M.DoRotX(grip_angle);
    //grpr_links_pose[2].p =  grpr_links_pose[2] *
    //                        KDL::Vector( 0.0, 0.0, link_dims_[2][2]/2);
    //grpr_links_pose[2].M.GetQuaternion(x, y, z, w);
    //
    //double link3_pose[7] = {grpr_links_pose[2].p.x(),
    //                        grpr_links_pose[2].p.y(), grpr_links_pose[2].p.z(), x, y, z, w};

//    gripper_links[2]->SetKinematicPose(link3_pose);
    hinges[1]->setMotorTarget(-grip_angle, 0.05);


//    //-------------------------------- LINKS 3 and 4
//    for (int i = 3; i < 5; ++i) {
//        // first find the end point of links 1 and 2 and then add half length
//        // of links 3 and 4
//        grpr_links_pose[i] = pose;
//        grpr_links_pose[i].p =
//                grpr_links_pose[i-2] *
//                KDL::Vector(0., 0.,link_dims_[i-2][2]/2)
//                + grpr_links_pose[i].M *
//                  KDL::Vector(0., 0.,link_dims_[i][2]/2);
//
//        grpr_links_pose[i].M.GetQuaternion(x,y,z,w);
//        double link_pose[7] = {grpr_links_pose[i].p.x(),
//                               grpr_links_pose[i].p.y(), grpr_links_pose[i].p.z(),x, y, z, w};
//
//        gripper_links[i]->SetKinematicPose(link_pose);
//    }


}

void Forceps::AddToWorld(btDiscreteDynamicsWorld * bt_world) {

    for (int i = 0; i < num_links_; ++i) {
        bt_world->addRigidBody(gripper_links[i]->GetBody());
    }

    for (int j = 0; j < sizeof(hinges)/sizeof(hinges[0]); ++j) {
        bt_world->addConstraint(hinges[j]);
    }


}

void Forceps::AddToActorsVector(
        std::vector<vtkSmartPointer<vtkProp>> &actors) {
    for (int i = 0; i < num_links_; ++i) {
        actors.push_back(gripper_links[i]->GetActor());
    }
}

bool Forceps::IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                                        btCollisionObject *obj) {

    MyContactResultCallback result0, result1;
    bt_world->contactPairTest(gripper_links[1]->GetBody(),
                              obj, result0);
    bool jaw1 = result0.connected;

    bt_world->contactPairTest(gripper_links[2]->GetBody(),
                              obj, result1);
    bool jaw2 = result1.connected;

    return (jaw1 & jaw2);

}

