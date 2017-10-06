//
// Created by nima on 09/08/17.
//

#ifndef ATAR_SIMPLEGRIPPER_H
#define ATAR_SIMPLEGRIPPER_H


#include "BulletVTKObject.h"
#include <kdl/frames.hpp>

class SimpleGripper {

public:
    SimpleGripper(const    std::vector<std::vector<double> > gripper_link_dims);

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

};


#endif //ATAR_SIMPLEGRIPPER_H
