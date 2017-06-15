//
// Created by nima on 15/06/17.
//

#ifndef ATAR_BULLETVTKOBJECT_H
#define ATAR_BULLETVTKOBJECT_H

#include "BulletVTKMotionState.h"
#include <btBulletDynamicsCommon.h>
#include <vector>

// the pose is array for easy initialization, make sure the size is 7 before
// passing in! {x, y, z, qx, qy, qz, qw}

enum ObjectType {DYNAMIC, KINEMATIC};
enum ObjectShape {SPHERE, BOX, CONE};

class BulletVTKObject {

public:
    BulletVTKObject(ObjectShape shape, ObjectType type, std::vector<double> dimensions,
                    double pose[], double mass);

    ~BulletVTKObject();

    btRigidBody* GetBody() { return body; }

    vtkSmartPointer<vtkActor> GetActor() {return actor;};


private:

    static uint num_bulletvtk_objects;

    btRigidBody* body;
    vtkSmartPointer<vtkActor> actor;
    BulletVTKMotionState* motion_state;
    btCollisionShape* collision_shape;

};

vtkSmartPointer<vtkMatrix4x4> PoseVectorToVTKMatrix(double pose[]);
#endif //ATAR_BULLETVTKOBJECT_H
