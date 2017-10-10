//
// Created by nima on 15/06/17.
//

#ifndef ATAR_SIMOBJECT_H
#define ATAR_SIMOBJECT_H

#include "BulletVTKMotionState.h"
#include <btBulletDynamicsCommon.h>
#include <vector>

/**
 * \class SimObject
 * \brief This class represents a simulated object with graphics and physics.
 * When a SimObject is generated its graphic actor (actor_) and physics body
 * (rigid_body_) has to be added to the renderer and physics world.
 * After which, the pose of the graphical representation will be
 * automatically updated by the physics simulation, thanks the
 * toBulletVTKMotionState member. To learn more about how to add simObjects
 * to the simulation world refer to VTKTask.h
 *
 * To construct a SimObject at least 5 arguments are needed.
 *
 * To get static objects make a dynamic one with zero mass.
 *
 * Meshes are decomposed into approximated compound meshes.
 *
 * IMPORTANT NOTE: We were interested in objects with dimensions in the
 * order of a few millimiters. It turned out that the bullet simulation
 * becomes unstable for such small dimensions. To get around this, the
 * dimensions of all the bullet related things are multiplied by B_DIM_SCALE.
 */




// -----------------------------------------------------------------------------
enum ObjectType {
    NOPHYSICS,  // Just graphics
    DYNAMIC,    // If density==0.0 object is STATIC
    KINEMATIC,   // Set the pose of the object externally
};

// -----------------------------------------------------------------------------
enum ObjectShape {
    STATICPLANE,    //dims = [normal_x, normal_y, normal_z]
    SPHERE,         //dims = [radius]
    CYLINDER,       //dims = [radius, height]
    BOX,            //dims = [width, length, height]
    CONE,           //dims = [radius, height]
    MESH
};

// -----------------------------------------------------------------------------
class SimObject {

public:
    SimObject(const ObjectShape shape, const ObjectType type,
              const std::vector<double> dimensions,
              const KDL::Frame &pose=KDL::Frame(),
              const double density=0.0, const double friction = 0.1,
              const std::string data = {}, const int id = 0);

    ~SimObject();

    btRigidBody* GetBody() { return rigid_body_; }

    vtkSmartPointer<vtkActor> GetActor() { return actor_; };

    void SetKinematicPose(double pose[]);

    int GetId() {return id_; };

    KDL::Frame GetPose();
private:

    int id_;
    ObjectType object_type_;
    btRigidBody* rigid_body_;
    vtkSmartPointer<vtkActor> actor_;
    BulletVTKMotionState* motion_state_;
    btCollisionShape* collision_shape_;

};

// -----------------------------------------------------------------------------
// helper functions
// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
vtkSmartPointer<vtkMatrix4x4> PoseArrayToVTKMatrix(double *pose);

vtkSmartPointer<vtkMatrix4x4> KDLFrameToVTKMatrix(const KDL::Frame &pose);

//KDL::Frame VTKMatrixToKDLFrame(const vtkSmartPointer<vtkMatrix4x4>);

// -----------------------------------------------------------------------------
// callback for pairwise collision test
struct MyContactResultCallback : public btCollisionWorld::ContactResultCallback
{
    bool connected;
    btScalar margin;
    MyContactResultCallback() :connected(false),
                               margin(0.001f*B_DIM_SCALE) {}

    virtual btScalar addSingleResult(btManifoldPoint& cp,
                                     const btCollisionObjectWrapper* colObj0Wrap,
                                     int partId0,int index0,
                                     const btCollisionObjectWrapper* colObj1Wrap,
                                     int partId1,int index1){
        if (cp.getDistance()<=margin)
            connected = true;
        return 1.f;
    }
};


// ----------------------------------------------------------------------------
#endif //ATAR_SIMOBJECT_H
