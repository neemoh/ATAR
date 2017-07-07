//
// Created by nima on 15/06/17.
//

#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <kdl/frames.hpp>
#include <vtkConeSource.h>
#include <vtkCylinderSource.h>
#include <src/ac_overlay_vtk/LoadObjGL/GLInstanceGraphicsShape.h>
#include <vtkOBJReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <iostream>
#include <sstream>
#include "BulletVTKObject.h"
#include "LoadObjGL/LoadMeshFromObj.h"
//for debug message
#include "ros/ros.h"


BulletVTKObject::BulletVTKObject(ObjectShape shape, ObjectType o_type,
                                 std::vector<double> dimensions,
                                 double pose[],
                                 double density,
                                 void *data,
                                 double friction,
                                 double contact_stiffness,
                                 double contact_damping
)
        : object_type_(o_type)
{

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    actor_ = vtkSmartPointer<vtkActor>::New();
    double volume = 0.0;
    std::string shape_string; // for debug report

    switch (shape){

        case ObjectShape::SPHERE : {
            // check if we have all the dimensions
            if (dimensions.size() != 1)
                throw std::runtime_error("BulletVTKObject BOX shape requires "
                                                 "a vector of 1 double "
                                                 "as dimensions.");
            // VTK actor_
            vtkSmartPointer<vtkSphereSource> source =
                    vtkSmartPointer<vtkSphereSource>::New();

            source->SetRadius(dimensions[0]);
            source->SetPhiResolution(30);
            source->SetThetaResolution(30);
            mapper->SetInputConnection(source->GetOutputPort());

            // Bullet Shape
            collision_shape_ = new btSphereShape(btScalar(B_DIM_SCALE*dimensions[0]));

            // calculate volume
            volume = 4/3*M_PI* pow(B_DIM_SCALE*dimensions[0], 3);

            // set name
            shape_string = collision_shape_->getName();

            break;
        }
// -------------------------------------------------------------------------
        case ObjectShape::CYLINDER : {
            // check if we have all the dimensions
            if (dimensions.size() != 2)
                throw std::runtime_error("BulletVTKObject CYLINDER shape requires "
                                                 "a vector of 1 double "
                                                 "as dimensions.");
            // VTK actor_
            vtkSmartPointer<vtkCylinderSource> source =
                    vtkSmartPointer<vtkCylinderSource>::New();

            source->SetRadius(dimensions[0]);
            source->SetHeight(dimensions[1]);
            source->SetResolution(30);
            mapper->SetInputConnection(source->GetOutputPort());

            // Bullet Shape
            // TODO: ATTENTION TO THE DOUBLED RADIUS
            collision_shape_ =
                    new btCylinderShape(btVector3(btScalar(B_DIM_SCALE*dimensions[0]),
                                                  btScalar(B_DIM_SCALE*dimensions[1]/2),
                                                  btScalar(B_DIM_SCALE*dimensions[0])
                    ));

            // calculate volume
            volume = M_PI * pow(B_DIM_SCALE*dimensions[0], 2) * B_DIM_SCALE*dimensions[1];

            // set name
            shape_string = collision_shape_->getName();

            break;
        }

        case ObjectShape::BOX : {

            // check if we have all dimensions
            if (dimensions.size() != 3)
                throw std::runtime_error("BulletVTKObject BOX shape requires "
                                                 "a vector of three doubles "
                                                 "as dimensions.");

            // VTK actor_
            vtkSmartPointer<vtkCubeSource> board_source =
                    vtkSmartPointer<vtkCubeSource>::New();

            board_source->SetXLength(dimensions[0]);
            board_source->SetYLength(dimensions[1]);
            board_source->SetZLength(dimensions[2]);

            mapper->SetInputConnection(board_source->GetOutputPort());

            // Bullet Shape
            collision_shape_ = new btBoxShape(
                    btVector3(btScalar(B_DIM_SCALE*dimensions[0]/2),
                              btScalar(B_DIM_SCALE*dimensions[1]/2),
                              btScalar(B_DIM_SCALE*dimensions[2]/2)));
            // calculate volume
            volume = B_DIM_SCALE*dimensions[0] *
                    B_DIM_SCALE*dimensions[1] *
                    B_DIM_SCALE*dimensions[2];

            // set name
            shape_string = collision_shape_->getName();;

            break;
        }
        case ObjectShape::CONE : {

            // check if we have all dimensions
            if (dimensions.size() != 2)
                throw std::runtime_error("BulletVTKObject CONE shape requires "
                                                 "a vector of two doubles "
                                                 "as dimensions.");

            // VTK actor_
            vtkSmartPointer<vtkConeSource> source =
                    vtkSmartPointer<vtkConeSource>::New();

            source->SetRadius(dimensions[0]);
            source->SetHeight(dimensions[1]);
            source->SetResolution(30);

            mapper->SetInputConnection(source->GetOutputPort());

            // Bullet Shape
            collision_shape_ = new btConeShape(
                    btScalar(B_DIM_SCALE*dimensions[0]), btScalar(B_DIM_SCALE*dimensions[1]/2));

            // calculate volume
            volume = float(M_PI* pow(B_DIM_SCALE*dimensions[0], 2) *
                                   B_DIM_SCALE*dimensions[1]/3);

            // set name
            shape_string = collision_shape_->getName();;

            break;
        }

        case ObjectShape::MESH : {

            std::string* filepath = static_cast<std::string*>(data);
            ROS_DEBUG("Loading mesh file from: %s", filepath->c_str()) ;
            //load our obj mesh

            GLInstanceGraphicsShape* glmesh = LoadMeshFromObj(filepath->c_str(), "");
            ROS_DEBUG("[INFO] Obj loaded: Extracted %d verticed from obj file "
                              "[%s]\n", glmesh->m_numvertices, filepath->c_str());

            const GLInstanceVertex& v = glmesh->m_vertices->at(0);
            btConvexHullShape* shape = new btConvexHullShape((const btScalar*)(&(v.xyzw[0])),
                                                             glmesh->m_numvertices,
                                                             sizeof(GLInstanceVertex));

            btVector3 localScaling(0.001,0.001,0.001);
            shape->setLocalScaling(localScaling);
//
            // option 1
//            shape->optimizeConvexHull();

            // option 2
            shape->initializePolyhedralFeatures();

            collision_shape_ = shape;
            //            //shape->setMargin(0.001);
            //            m_collisionShapes.push_back(shape);
            //
            //            btTransform startTransform;
            //            startTransform.setIdentity();
            //
            //            btScalar	mass(1.f);
            //            bool isDynamic = (mass != 0.f);
            //            btVector3 localInertia(0,0,0);
            //            if (isDynamic)
            //                shape->calculateLocalInertia(mass,localInertia);
            //
            //            float color[4] = {1,1,1,1};
            //            float orn[4] = {0,0,0,1};
            //            float pos[4] = {0,3,0,0};
            //            btVector3 position(pos[0],pos[1],pos[2]);
            //            startTransform.setOrigin(position);
            //            btRigidBody* body = createRigidBody(mass,startTransform,shape);

            // -------------------------------------------------------------------------
            // VTK


            vtkSmartPointer<vtkOBJReader> reader =
                    vtkSmartPointer<vtkOBJReader>::New();

            reader->SetFileName(filepath->c_str());

            reader->Update();
            mapper->SetInputConnection(reader->GetOutputPort());

            // calculate the volume
            vtkSmartPointer<vtkMassProperties> Mass =
                    vtkSmartPointer<vtkMassProperties>::New();
            vtkSmartPointer<vtkTriangleFilter> tri_filt =
                    vtkSmartPointer<vtkTriangleFilter>::New();
            tri_filt->SetInputConnection(reader->GetOutputPort());
            Mass->SetInputConnection(tri_filt->GetOutputPort());
            volume = Mass->GetVolume();
            //            // transform would be necesary at this stage if
            // cellLocator is going to be used
            //            vtkSmartPointer<vtkTransform> transform =
            //                    vtkSmartPointer<vtkTransform>::New();
            //            transform->Translate(0.050, 0.060, 0.025);
            //            transform->RotateX(180);
            //            transform->RotateZ(150);
            //
            //            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
            //                    vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            //            transformFilter->SetInputConnection(
            //                    reader->GetOutputPort());
            //            transformFilter->SetTransform(transform);
            //            transformFilter->Update();
            //
            //            vtkSmartPointer<vtkPolyDataMapper> mapper =
            //                    vtkSmartPointer<vtkPolyDataMapper>::New();
            //            mapper->SetInputConnection(
            //                    transformFilter->GetOutputPort());

            break;
        }
    }

    actor_->SetMapper(mapper);

    //------------------------------------------------------------------------------
    // set up dynamics
    // rigid body_ is dynamic if and only if mass is non zero, otherwise static

    if(object_type_!=NOPHYSICS) {

        btScalar bt_mass = float(volume * density);

        bool isStatic = (bt_mass == 0.f);
        btVector3 local_inertia(0, 0, 0);

        if (isStatic)
            actor_->SetUserMatrix(PoseVectorToVTKMatrix(pose));

        if (!isStatic && (object_type_ != ObjectType::KINEMATIC))
            // Set initial pose of graphical representation
            collision_shape_->calculateLocalInertia(bt_mass, local_inertia);

        // ensure zero mass when Kinematic
        if (object_type_ == ObjectType::KINEMATIC)
            bt_mass = 0.f;

        // construct a motion state to connect the pose of the graphical
        // representation to that of the dynamic one
        motion_state_ = new BulletVTKMotionState(pose, actor_);

        // construct body_ info
        btRigidBody::btRigidBodyConstructionInfo body_info(
                bt_mass, motion_state_, collision_shape_, local_inertia);
//        body_info.m_restitution = (btScalar) restitution;
//        body_info.m_friction = (btScalar) friction;

        body_ = new btRigidBody(body_info);

        // set appropriate flags if the body is kinematic
        if (object_type_ == ObjectType::KINEMATIC) {
            body_->setCollisionFlags(body_->getCollisionFlags() |
                                     btCollisionObject::CF_KINEMATIC_OBJECT);
            body_->setActivationState(DISABLE_DEACTIVATION);
        }

//

        // to prevent rounded objects from rolling for ever we add a bit of
        // rolling friction
        if (shape == ObjectShape::CONE ||
            shape == ObjectShape::CYLINDER){
            body_->setRollingFriction(0.02);
            body_->setSpinningFriction(0.02);
//            body_->setAnisotropicFriction
//                    (collision_shape_->getAnisotropicRollingFrictionDirection
//                            (),btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
        }
        else if(shape == ObjectShape::SPHERE) {
            body_->setRollingFriction(0.001);
            body_->setSpinningFriction(0.001);
        }

        body_->setFriction((btScalar)friction);
        body_->setSpinningFriction(0.001);

        //set contact parameters
//        body_->setContactStiffnessAndDamping((float) contact_stiffness,
//                                             (float) contact_damping);
        std::stringstream debug_msg;

        debug_msg << std::string(" shape = ") << shape_string
                  << ", mass = " << bt_mass
                  << ", volume = " << volume
                  << ", friction = " << friction
                  << ", contact_stiffness = " << contact_stiffness
                  << ", contact_damping = " << contact_damping;
        ROS_DEBUG("Created BulletVTKObject with properties: %s", debug_msg.str()
                .c_str());
    }
}


//------------------------------------------------------------------------------
BulletVTKObject::~BulletVTKObject() {
// deleting from outside
//    delete collision_shape_;
//    delete motion_state_;
//    delete body_;
}


//------------------------------------------------------------------------------
void BulletVTKObject::SetKinematicPose(double *pose) {

    if(object_type_==KINEMATIC){

        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(float(B_DIM_SCALE*pose[0]),
                                      float(B_DIM_SCALE*pose[1]),
                                      float(B_DIM_SCALE*pose[2]) ));;
        transform.setRotation(btQuaternion((float) pose[3], (float) pose[4],
                                           (float) pose[5], (float) pose[6]));

        motion_state_->setKinematicPos(transform);
//        body_->setWorldTransform(transform);
    }
}


//------------------------------------------------------------------------------
vtkSmartPointer<vtkMatrix4x4> PoseVectorToVTKMatrix(double pose[]) {

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

    return out;
}


