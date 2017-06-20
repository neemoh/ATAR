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

// Bullet meshes can only be used as static colliders (terrain for
// instance), not for dynamic bodies. For that you should either use Convex
// Hulls and/or basic shapes to create bodies that resemble your mesh to some
// extent. Please see ConvexDecomposition demo for that.

enum ObjectType {
    NOPHYSICS,  // Just graphics
    DYNAMIC,    // If density==0.0 object is STATIC
    KINEMATIC   // Set the pose of the object externally
};

enum ObjectShape {
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
                    double friction = 0.5,
                    double contact_stiffness = 10000.0,
                    double contact_damping = 0.0
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
