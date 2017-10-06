//
// Created by nima on 10/08/17.
//

#ifndef ATAR_THREELINKGRIPPER_H
#define ATAR_THREELINKGRIPPER_H

#include "BulletVTKObject.h"
#include <kdl/frames.hpp>

class ThreeLinkGripper {

public:
    ThreeLinkGripper(const    std::vector<std::vector<double> > gripper_link_dims);

    void SetPoseAndJawAngle(const KDL::Frame pose,
                            const double grip_angle);

    uint GetNumLinks(){ return num_links_;};

    void AddToWorld(btDiscreteDynamicsWorld* bt_world);

    void AddToActorsVector(std::vector<vtkSmartPointer<vtkProp>> & actors);

    bool IsGraspingObject(btDiscreteDynamicsWorld* bt_world,
                          btCollisionObject* obj);

private:

    uint num_links_;

    std::vector<std::vector<double> > link_dims_;

    BulletVTKObject* gripper_links[5];

    btHingeConstraint* hinges[2];
};



#endif //ATAR_THREELINKGRIPPER_H
