//
// Created by nima on 09/08/17.
//

#include "SimFiveLinkGripper.h"
#include <vtkProperty.h>

SimFiveLinkGripper::SimFiveLinkGripper(
        const std::vector<std::vector<double> > link_dims_)
        : link_dims_(link_dims_)
{


    float gripper_density = 0; // kg/m3
    float gripper_friction = 50;

    for (int i = 0; i < 5; ++i) {

        sim_objects_.emplace_back(new SimObject(ObjectShape::BOX, ObjectType::KINEMATIC,
                              link_dims_[i], KDL::Frame(), gripper_density,
                              gripper_friction));
        sim_objects_[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
        sim_objects_[i]->GetActor()->GetProperty()->SetSpecularPower(50);
        sim_objects_[i]->GetActor()->GetProperty()->SetSpecular(0.8);

        sim_objects_[i]->GetBody()->setContactStiffnessAndDamping
                (2000, 100);
    }
}

void SimFiveLinkGripper::SetPoseAndJawAngle(const KDL::Frame pose,
                                       const double grip_angle) {

    KDL::Frame grpr_links_pose[5];

    //-------------------------------- LINK 0
    grpr_links_pose[0] = pose;
    grpr_links_pose[0].p  = grpr_links_pose[0] * KDL::Vector( 0.0 , 0.0,
                                                              -link_dims_[0][2]/2);
    sim_objects_[0]->SetKinematicPose(grpr_links_pose[0]);

    //-------------------------------- LINK 1
    grpr_links_pose[1] = pose;
    grpr_links_pose[1].M.DoRotX(-grip_angle);
    grpr_links_pose[1].p =  grpr_links_pose[1] *
                            KDL::Vector( 0.0, 0.0, link_dims_[1][2]/2);
    sim_objects_[1]->SetKinematicPose(grpr_links_pose[1]);

    //-------------------------------- LINK 2
    grpr_links_pose[2] = pose;
    grpr_links_pose[2].M.DoRotX(grip_angle);
    grpr_links_pose[2].p =  grpr_links_pose[2] *
                            KDL::Vector( 0.0, 0.0, link_dims_[2][2]/2);
    sim_objects_[2]->SetKinematicPose(grpr_links_pose[2]);

    //-------------------------------- LINKS 3 and 4
    for (int i = 3; i < 5; ++i) {
        // first find the end point of links 1 and 2 and then add half length
        // of links 3 and 4
        grpr_links_pose[i] = pose;
        grpr_links_pose[i].p =
                grpr_links_pose[i-2] *
                KDL::Vector(0., 0.,link_dims_[i-2][2]/2)
                + grpr_links_pose[i].M *
                  KDL::Vector(0., 0.,link_dims_[i][2]/2);
        sim_objects_[i]->SetKinematicPose(grpr_links_pose[i]);
    }
}

bool SimFiveLinkGripper::IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                                     btCollisionObject *obj) {

    MyContactResultCallback result0, result1;
    bt_world->contactPairTest(sim_objects_[3]->GetBody(),
                              obj, result0);
    bool jaw1 = result0.connected;

    bt_world->contactPairTest(sim_objects_[4]->GetBody(),
                              obj, result1);
    bool jaw2 = result1.connected;

    return (jaw1 & jaw2);

}

