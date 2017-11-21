//
// Created by nima on 12/08/17.
//

#ifndef ATAR_FORCEPS_H
#define ATAR_FORCEPS_H

#include "SimObject.h"
#include "SimMechanism.h"
#include <kdl/frames.hpp>

class SimForceps : public SimMechanism{

public:
    explicit SimForceps(KDL::Frame init_pose=KDL::Frame());

    void SetPoseAndJawAngle(KDL::Frame pose,
                            double grip_angle);

    bool IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                          btCollisionObject* obj);

private:
    std::vector<std::vector<double> > link_dims_;

};



#endif //ATAR_FORCEPS_H
