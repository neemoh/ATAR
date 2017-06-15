//
// Created by nima on 14/06/17.
//

#ifndef ATAR_BULLETVTKMOTIONSTATE_H
#define ATAR_BULLETVTKMOTIONSTATE_H


#include <LinearMath/btMotionState.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>

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

    \author    Nima Enayati

*/
//==============================================================================


class BulletVTKMotionState : public btMotionState{

protected:
    vtkSmartPointer<vtkActor>   actor_;
    btTransform                 bt_pose_;

public:
    BulletVTKMotionState(const btTransform &initialPosition,
                         vtkSmartPointer<vtkActor> actor){
        actor_ = actor;
        bt_pose_ = initialPosition;
    }


    // -------------------------------------------------------------------------
    virtual ~BulletVTKMotionState(){}


    // -------------------------------------------------------------------------
    //! This shouold not really be needed!
    void setActor(vtkSmartPointer<vtkActor> actor){ actor_ = actor; }


    // -------------------------------------------------------------------------
    //! called by bullet at initialization for all objects and if kinematic
    //! object it is called at each loop
    virtual void getWorldTransform(btTransform &worldTrans) const {
        worldTrans = bt_pose_; }


    // -------------------------------------------------------------------------
    //! Called by bullet to set the pose of dynamic objects
    virtual void setWorldTransform(const btTransform &worldTrans) {

        if(actor_ == nullptr)
            return; // silently return

        btQuaternion rot = worldTrans.getRotation();
        // TODO
        //        actor_->SetOrientation(rot.w(), rot.x(), rot.y(), rot.z());
        actor_->SetOrientation(0.0, 0.0, 0.0);
        btVector3 pos = worldTrans.getOrigin();
        actor_->SetPosition(pos.x(), pos.y(), pos.z());
    }


    // -------------------------------------------------------------------------
    //! Used by the user to set the pose of kinematic object
    void setKinematicPos(btTransform &currentPos) { bt_pose_ = currentPos; }

};

#endif //ATAR_BULLETVTKMOTIONSTATE_H
