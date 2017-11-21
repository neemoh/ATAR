//
// Created by nima on 10/08/17.
//

#include "SimGripperLarge.h"
#include <vtkProperty.h>
extern std::string                      RESOURCES_DIRECTORY;

SimGripperLarge::SimGripperLarge(const KDL::Frame init_pose)
{

    auto jaws_axis_y_offset = -0.003f;
    auto link0_axis_z_offset = 0.006f;

    link_dims_ = std::vector<double>({0.003, 0.003, 0.003});
    float gripper_density = 50000; // kg/m3
    float gripper_friction = 50;

    // create the kinematic link
    sim_objects_.emplace_back(new SimObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                                             link_dims_, init_pose, 0.0, 0));
    sim_objects_[0]->GetActor()->GetProperty()->SetColor(0.7f, 0.7f, 0.7f);

    // create jaw 1
    sim_objects_.emplace_back(new SimObject(ObjectShape::MESH,
                                            ObjectType::DYNAMIC,
                                            RESOURCES_DIRECTORY+
                                                    "/mesh/jaw_large.obj",
                                            init_pose,
                                            gripper_density,
                                            gripper_friction));
    sim_objects_[1]->GetActor()->GetProperty()->SetColor(1.0,  0.35, 0.);
    sim_objects_[1]->GetBody()->setContactStiffnessAndDamping(500, 100);
    sim_objects_[1]->GetBody()->setRollingFriction(btScalar(0.1));
    sim_objects_[1]->GetBody()->setSpinningFriction(btScalar(0.1));

    // create jaw 2

    KDL::Rotation jaw_2_rot;
    jaw_2_rot.DoRotZ(M_PI);
    KDL::Frame gripper_pose;
    gripper_pose.M = init_pose.M * jaw_2_rot;
    gripper_pose.p = init_pose*KDL::Vector(0.f, jaws_axis_y_offset,
                                                  -link0_axis_z_offset);
    sim_objects_.emplace_back(new SimObject(ObjectShape::MESH,
                                            ObjectType::DYNAMIC,
                                            RESOURCES_DIRECTORY+
                                                    "/mesh/jaw_large.obj",
                                            gripper_pose,
                                            gripper_density,
                                            gripper_friction));
    sim_objects_[2]->GetActor()->GetProperty()->SetColor(1.0,  0.35, 0.);
    sim_objects_[2]->GetBody()->setContactStiffnessAndDamping(500, 100);
    sim_objects_[2]->GetBody()->setRollingFriction(btScalar(0.1));

    const btVector3 pivot_link0(0.f, 0.f, link0_axis_z_offset  * B_DIM_SCALE);
    const btVector3 pivot_jaw_1(0.f, jaws_axis_y_offset* B_DIM_SCALE, 0.f);
    const btVector3 pivot_jaw_2(0.f, jaws_axis_y_offset* B_DIM_SCALE, 0.f);

    btVector3 btAxisA( 1.0f, 0.f, 0.0f );
    btVector3 btAxisB( 0.0f, -1.f, 0.f );
    btVector3 btAxisC( 0.0f, 1.f, 0.f );

    constraints_.emplace_back( new btHingeConstraint(
            *sim_objects_[0]->GetBody(),
            *sim_objects_[1]->GetBody(),
            pivot_link0, pivot_jaw_1, btAxisA, btAxisB ));
    constraints_[0]->enableMotor(true);
    constraints_[0]->setMaxMotorImpulse(200);
    //    constraints_[0]->setLimit(0, M_PI/4);

    constraints_.emplace_back(new btHingeConstraint(
            *sim_objects_[0]->GetBody(),
            *sim_objects_[2]->GetBody(),
            pivot_link0, pivot_jaw_2, btAxisA, btAxisC ));
    constraints_[1]->enableMotor(true);
    constraints_[1]->setMaxMotorImpulse(200);
}

void SimGripperLarge::SetPoseAndJawAngle(const KDL::Frame pose,
                                 const double grip_angle) {

    KDL::Frame link_pose = pose;
    link_pose.p  = link_pose * KDL::Vector( 0.0 , 0.0, -link_dims_[2]/2);

    sim_objects_[0]->SetKinematicPose(link_pose);

    constraints_[0]->setMotorTarget((btScalar)grip_angle, 0.09);

    constraints_[1]->setMotorTarget((btScalar)-grip_angle, 0.09);

}


bool SimGripperLarge::IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                               btCollisionObject *obj) {

    MyContactResultCallback result0, result1;
    bt_world->contactPairTest(sim_objects_[1]->GetBody(),
                              obj, result0);
    bool jaw1 = result0.connected;

    bt_world->contactPairTest(sim_objects_[2]->GetBody(),
                              obj, result1);
    bool jaw2 = result1.connected;

    return (jaw1 & jaw2);

}


