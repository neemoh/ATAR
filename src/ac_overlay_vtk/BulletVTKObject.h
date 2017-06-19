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
// To get statis objects make a dynamic one with sero mass.

enum ObjectType {NOPHYSICS, DYNAMIC, KINEMATIC};
enum ObjectShape {SPHERE, BOX, CONE};

class BulletVTKObject {

public:
    BulletVTKObject(ObjectShape shape, ObjectType type, std::vector<double> dimensions,
                    double pose[],
                    double density,
                    double contact_stiffness = 10000.0,
                    double contact_damping = 0.0,
                    double friction = 0.0,
                    double restitution = 0.0
                    );

    ~BulletVTKObject();

    btRigidBody* GetBody() { return body_; }

    vtkSmartPointer<vtkActor> GetActor() { return actor_; };

    void SetKinematicPose(double pose[]);

private:

    static uint num_bulletvtk_objects;
    ObjectType object_type_;
    btRigidBody* body_;
    vtkSmartPointer<vtkActor> actor_;
    BulletVTKMotionState* motion_state_;
    btCollisionShape* collision_shape_;

};

vtkSmartPointer<vtkMatrix4x4> PoseVectorToVTKMatrix(double pose[]);
#endif //ATAR_BULLETVTKOBJECT_H
