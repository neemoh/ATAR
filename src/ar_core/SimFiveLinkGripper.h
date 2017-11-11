//
// Created by nima on 09/08/17.
//

#ifndef ATAR_SIMPLEGRIPPER_H
#define ATAR_SIMPLEGRIPPER_H


#include "SimObject.h"
#include "SimMechanism.h"
#include <kdl/frames.hpp>

class SimFiveLinkGripper: public SimMechanism {

public:
    SimFiveLinkGripper(const    std::vector<std::vector<double> > gripper_link_dims);

    void SetPoseAndJawAngle(const KDL::Frame pose,
                                const double grip_angle);

    bool IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                          btCollisionObject* obj);

private:


    std::vector<std::vector<double> > link_dims_;

};


#endif //ATAR_SIMPLEGRIPPER_H
