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
// To get static objects make a dynamic one with sero mass.

// Meshes are decomposed into approximated compound meshes.
//
//IMPORTANT NOTE: We were interested in objects with dimensions in the
//        order of a few millimiters. It turned out that the bullet simulation
//        becomes unstable for such small dimensions. To get around this, the
//        dimensions of all the bullet related things are multiplied by B_DIM_SCALE.

enum ObjectType {
    NOPHYSICS,  // Just graphics
    DYNAMIC,    // If density==0.0 object is STATIC
    KINEMATIC,   // Set the pose of the object externally
};

enum ObjectShape {
    STATICPLANE, //dims = [normal_x, normal_y, normal_z]
    SPHERE,     //dims = [radius]
    CYLINDER,   //dims = [radius, height]
    BOX,        //dims = [width, length, height]
    CONE,       //dims = [radius, height]
    MESH};

class BulletVTKObject {

public:
    BulletVTKObject(ObjectShape shape,
                    ObjectType type,
                    std::vector<double> dimensions,
                    double pose[],
                    double density,
                    const int id = 0,
                    double friction = 0.1,
                    void *data = NULL);

    ~BulletVTKObject();

    btRigidBody* GetBody() { return rigid_body_; }

    vtkSmartPointer<vtkActor> GetActor() { return actor_; };

    void SetKinematicPose(double pose[]);

    int GetId() {return id_; };
private:

    int id_;
    ObjectType object_type_;
    btRigidBody* rigid_body_;
    vtkSmartPointer<vtkActor> actor_;
    BulletVTKMotionState* motion_state_;
    btCollisionShape* collision_shape_;

};

vtkSmartPointer<vtkMatrix4x4> PoseVectorToVTKMatrix(double pose[]);
#endif //ATAR_BULLETVTKOBJECT_H
