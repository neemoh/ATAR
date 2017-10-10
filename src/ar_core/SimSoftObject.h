//
// Created by nima on 04/08/17.
//

#ifndef ATAR_BULLETVTKSOFTOBJECT_H
#define ATAR_BULLETVTKSOFTOBJECT_H

#include "BulletVTKMotionState.h"
#include <btBulletDynamicsCommon.h>
#include <vector>
#include <BulletSoftBody/btSoftBody.h>

class SimSoftObject {

public:
    SimSoftObject(btSoftBodyWorldInfo &world_info,
                        const std::string mesh_file_dir,
                        KDL::Frame pose, float density,
                        float friction=0.1);

    ~SimSoftObject();

    btSoftBody* GetBody() { return body_; }

    vtkSmartPointer<vtkActor> GetActor() { return actor_; };

//    void SetKinematicPose(double pose[]);

    void RenderSoftbody();

private:

    btSoftBody *body_;
    vtkSmartPointer<vtkActor> actor_;
    btCollisionShape* collision_shape_;

};


#endif //ATAR_BULLETVTKSOFTOBJECT_H
