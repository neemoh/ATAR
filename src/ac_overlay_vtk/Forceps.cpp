//
// Created by nima on 10/08/17.
//

#include "Forceps.h"
#include <vtkProperty.h>
#include <sstream>

Forceps::Forceps(const std::string mesh_dir, const KDL::Frame init_pose)
        : num_links_(3)
{

    auto jaws_axis_y_offset = -0.001f;
    auto link0_axis_z_offset = 0.004f;

    link_dims_.push_back({0.001, 0.003, 0.003});
    float gripper_density = 500000; // kg/m3
    float gripper_friction = 50;

    double qx, qy, qz, qw;
    init_pose.M.GetQuaternion(qx, qy, qz, qw);

    // create the kinematic link
    {
        double gripper_pose[7]{init_pose.p[0], init_pose.p[1],
                               init_pose.p[2], qx, qy, qz, qw};


        gripper_links[0] =
                new BulletVTKObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                    link_dims_[0], gripper_pose,
                                    0.0, 0);
        gripper_links[0]->GetActor()->GetProperty()->SetColor(0.7f, 0.7f,
                                                              0.7f);
    }

    std::stringstream input_file_dir;
    input_file_dir << mesh_dir
                   << std::string("jaw.obj");
    std::string mesh_file_dir_str = input_file_dir.str();


    // create jaw 1
    {

        auto gripper_pose_position = init_pose*KDL::Vector(0.f,
                                                           -jaws_axis_y_offset,
                                                           -link0_axis_z_offset);
        double gripper_pose[7]{gripper_pose_position.x(),
                               gripper_pose_position.y(),
                               gripper_pose_position.z(),
                               qx, qy, qz, qw};

        gripper_links[1] =
                new BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                                    link_dims_[0], gripper_pose,
                                    gripper_density, 1, gripper_friction,
                                    &mesh_file_dir_str);
        gripper_links[1]->GetActor()->GetProperty()->SetColor(0.7f, 0.7f, 0.7f);
        gripper_links[1]->GetBody()->setContactStiffnessAndDamping(1000, 100);
        gripper_links[1]->GetBody()->setRollingFriction(btScalar(0.1));
        gripper_links[1]->GetBody()->setSpinningFriction(btScalar(0.1));
    }

    // create jaw 2
    {

        auto gripper_pose_position =
                init_pose*KDL::Vector(0.f, jaws_axis_y_offset,
                                      -link0_axis_z_offset);
        KDL::Rotation jaw_2_rot;
        jaw_2_rot.DoRotZ(M_PI);
        jaw_2_rot = init_pose.M * jaw_2_rot;
        jaw_2_rot.GetQuaternion(qx, qy, qz, qw);

        double gripper_pose[7]{gripper_pose_position.x(),
                               gripper_pose_position.y(),
                               gripper_pose_position.z(),
                               qx, qy, qz, qw};
        gripper_links[2] =
            new BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                                link_dims_[0], gripper_pose,
                                gripper_density, 1, gripper_friction,
                                &mesh_file_dir_str);
        gripper_links[2]->GetActor()->GetProperty()->SetColor(0.7f, 0.7f, 0.7f);
        gripper_links[2]->GetBody()->setContactStiffnessAndDamping(1000, 100);
        gripper_links[2]->GetBody()->setRollingFriction(btScalar(0.1));
        gripper_links[2]->GetBody()->setSpinningFriction(btScalar(0.1));
    }


    const btVector3 pivot_link0(0.f, 0.f, link0_axis_z_offset  * B_DIM_SCALE);
    const btVector3 pivot_jaw_1(0.f, jaws_axis_y_offset* B_DIM_SCALE, 0.f);
    const btVector3 pivot_jaw_2(0.f, jaws_axis_y_offset* B_DIM_SCALE, 0.f);

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

    hinges[0]->setMotorTarget(grip_angle, 0.08);

    hinges[1]->setMotorTarget(-grip_angle, 0.08);



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

