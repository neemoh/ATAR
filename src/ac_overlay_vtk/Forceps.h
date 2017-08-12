//
// Created by nima on 12/08/17.
//

#ifndef ATAR_FORCEPS_H
#define ATAR_FORCEPS_H

#include "BulletVTKObject.h"
#include <kdl/frames.hpp>

class Forceps {

public:
    Forceps(const std::string mesh_dir);

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



#endif //ATAR_FORCEPS_H
