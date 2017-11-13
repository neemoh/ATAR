//
// Created by nima on 14/06/17.
//

#ifndef ATAR_BULLETVTKMOTIONSTATE_H
#define ATAR_BULLETVTKMOTIONSTATE_H


#include <LinearMath/btMotionState.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <kdl/frames.hpp>
#include "VTKConversions.h"

#define B_DIM_SCALE 100.0f
//==============================================================================
/*!
    \class      BulletVTKMotionState
    \brief
    This class simplifies the graphics and dynamic objects synchronization.
    \details
	If the dynamic object moves the setWorldTransform will be called by
    bullet and we set the pose of the vtk actor.
    If the object is kinematic (e.g. a tool)the getWorldTransform method
    is called at every loop, and the pose of the object must be set
    externally using the setKinematicPos method.

    IMPORTANT NOTE: We were interested in objects with dimensions in the
    order of a few mm. It turned out that the bullet simulation
    becomes unstable for such small dimensions. To get around this, the
    dimensions of all the bullet related things are multiplied by B_DIM_SCALE.

    \author    Nima Enayati

*/


class BulletVTKMotionState : public btMotionState{

protected:
    vtkSmartPointer<vtkActor>   actor_;
    btTransform                 bt_pose_;
    KDL::Frame                  frame;

public:
    BulletVTKMotionState(const KDL::Frame &pose,
                         vtkSmartPointer<vtkActor> actor)
            : actor_(actor){

        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3(float(B_DIM_SCALE*pose.p[0]),
                                           float(B_DIM_SCALE*pose.p[1]),
                                           float(B_DIM_SCALE*pose.p[2]) ));
        double qx, qy,qz, qw;
        pose.M.GetQuaternion(qx, qy,qz, qw);
        init_transform.setRotation(btQuaternion((float)qx, (float)qy,
                                                (float)qz, (float)qw ));
        bt_pose_ = init_transform;
    }

    // -------------------------------------------------------------------------
    //! called by bullet at initialization for all objects and if kinematic
    //! object it is called at each loop
    void getWorldTransform(btTransform &worldTrans) const override {
        worldTrans = bt_pose_;
    }


    // -------------------------------------------------------------------------
    //! called by user to get the pose of the object
    KDL::Frame getKDLFrame() {
        return frame;
    }

    // -------------------------------------------------------------------------
    //! Called by bullet to set the pose of dynamic objects
    void setWorldTransform(const btTransform &worldTrans) override {

        // bullet
        bt_pose_ = worldTrans;
        frame = btTransformToKDLFrame(bt_pose_);

        // VTK
        vtkSmartPointer<vtkMatrix4x4> v_m =vtkSmartPointer<vtkMatrix4x4>::New();
        VTKConversions::KDLFrameToVTKMatrix(frame, v_m);
        actor_->SetUserMatrix(v_m);
    }

    // -------------------------------------------------------------------------
    //! Used by the user to set the pose of kinematic object
    void setKinematicPos(const KDL::Frame &in) {

        // Bullet side
        bt_pose_ = KDLFrameToBtTransform(in);

        // VTK side
        frame = in;
        vtkSmartPointer<vtkMatrix4x4> v_m =vtkSmartPointer<vtkMatrix4x4>::New();
        VTKConversions::KDLFrameToVTKMatrix(in, v_m);
        actor_->SetUserMatrix(v_m);
    }

private:

    btTransform KDLFrameToBtTransform(const KDL::Frame &in){
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(B_DIM_SCALE*in.p[0]),
                                      btScalar(B_DIM_SCALE*in.p[1]),
                                      btScalar(B_DIM_SCALE*in.p[2]) ));;
        double qx, qy, qz, qw;
        in.M.GetQuaternion(qx, qy, qz, qw);

        transform.setRotation(btQuaternion((btScalar) qx, (btScalar) qy,
                                           (btScalar) qz, (btScalar) qw));
        return transform;
    }

    KDL::Frame btTransformToKDLFrame(btTransform in){
        btQuaternion rot = in.getRotation();
        btVector3 pos = in.getOrigin();

        return KDL::Frame(
                KDL::Rotation::Quaternion( rot.x(),  rot.y(), rot.z(), rot.w()),
                KDL::Vector(pos.x(), pos.y(), pos.z())/B_DIM_SCALE);
    }
};





#endif //ATAR_BULLETVTKMOTIONSTATE_H
