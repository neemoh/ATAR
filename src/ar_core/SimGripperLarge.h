//
// Created by nima on 12/08/17.
//

#ifndef ATAR_GRIPPERLARGE_H
#define ATAR_GRIPPERLARGE_H

#include "SimObject.h"
#include "SimMechanism.h"
#include <kdl/frames.hpp>

class SimGripperLarge : public SimMechanism{

public:
    explicit SimGripperLarge(KDL::Frame init_pose=KDL::Frame());

    void SetPoseAndJawAngle(KDL::Frame pose,
                            double grip_angle);

    bool IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                          btCollisionObject* obj);

private:
    std::vector<std::vector<double> > link_dims_;

};



#endif //ATAR_GRIPPERLARGE_H
