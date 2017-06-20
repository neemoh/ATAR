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
// TODO FIX THIS DUPLICATED FUNCTION
//------------------------------------------------------------------------------

class BulletVTKMotionState : public btMotionState{

protected:
    vtkSmartPointer<vtkActor>   actor_;
    btTransform                 bt_pose_;

public:
    BulletVTKMotionState(const double pose[],
                         vtkSmartPointer<vtkActor> actor){

        btTransform init_transform;
        init_transform.setIdentity();
        init_transform.setOrigin(btVector3((float) pose[0], (float) pose[1],
                                           (float) pose[2]));
        init_transform.setRotation(btQuaternion((float) pose[3], (float) pose[4],
                                                (float) pose[5], (float) pose[6]));
        actor_ = actor;
        bt_pose_ = init_transform;
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
        worldTrans = bt_pose_;
    }


    // -------------------------------------------------------------------------
    //! Called by bullet to set the pose of dynamic objects
    virtual void setWorldTransform(const btTransform &worldTrans) {

        bt_pose_ = worldTrans;

        // Set the orientation
        btQuaternion rot = worldTrans.getRotation();
        btVector3 pos = worldTrans.getOrigin();

        //KDL::Rotation rot_kdl =
        //        KDL::Rotation::Quaternion(rot.x(), rot.y(), rot.z(), rot.w());
        //double r,p,y;
        //rot_kdl.GetRPY(r, p,y);
        //actor_->SetOrientation(r*180/M_PI, p*180/M_PI, y*180/M_PI);
        double pose[7] = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};

        vtkSmartPointer<vtkMatrix4x4> out =
            vtkSmartPointer<vtkMatrix4x4>::New();

        KDL::Frame k(KDL::Rotation::Quaternion( pose[3],  pose[4],
                                                pose[5],  pose[6])
            , KDL::Vector(pose[0], pose[1], pose[2]) );

        // Convert to VTK matrix.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out->SetElement(i, j, k.M(i,j));
            }
            out->SetElement(i, 3, k.p[i]);
        }

        actor_->SetUserMatrix(out);
        // set the position
        //actor_->SetPosition(pos.x(), pos.y(), pos.z());

    }


    // -------------------------------------------------------------------------
    //! Used by the user to set the pose of kinematic object
    void setKinematicPos(btTransform &currentPos) {

        bt_pose_ = currentPos;
        btQuaternion rot = bt_pose_.getRotation();
        btVector3 pos = bt_pose_.getOrigin();

        //KDL::Rotation rot_kdl =
        //        KDL::Rotation::Quaternion(rot.x(), rot.y(), rot.z(), rot.w());
        //double r,p,y;
        //rot_kdl.GetRPY(r, p,y);
        //actor_->SetOrientation(r*180/M_PI, p*180/M_PI, y*180/M_PI);
        double pose[7] = {pos.x(), pos.y(), pos.z(), rot.x(), rot.y(), rot.z(), rot.w()};

        vtkSmartPointer<vtkMatrix4x4> out =
            vtkSmartPointer<vtkMatrix4x4>::New();

        KDL::Frame k(KDL::Rotation::Quaternion( pose[3],  pose[4],
                                                pose[5],  pose[6])
            , KDL::Vector(pose[0], pose[1], pose[2]) );

        // Convert to VTK matrix.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out->SetElement(i, j, k.M(i,j));
            }
            out->SetElement(i, 3, k.p[i]);
        }

        actor_->SetUserMatrix(out);

    }

};


#endif //ATAR_BULLETVTKMOTIONSTATE_H
