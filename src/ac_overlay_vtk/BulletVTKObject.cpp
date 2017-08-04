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
#include <sys/stat.h>

inline bool FileExists (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

BulletVTKObject::BulletVTKObject(
        ObjectShape shape,
        ObjectType o_type,
        std::vector<double> dimensions,
        double pose[],
        double density,
        void *data,
        double friction
    )
        : object_type_(o_type)
{


    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    actor_ = vtkSmartPointer<vtkActor>::New();
    double volume = 0.0;
    std::string shape_string; // for debug report

    switch (shape){

        case STATICPLANE : {
            // check if we have all the dimensions
            if (dimensions.size() != 4)
                throw std::runtime_error("BulletVTKObject STATICPLANE "
                                             "shape requires a vector of 4 "
                                             "doubles as dimensions.");

            collision_shape_ = new btStaticPlaneShape(
                btVector3(btScalar(B_DIM_SCALE*dimensions[0]),
                          btScalar(B_DIM_SCALE*dimensions[1]),
                          btScalar(B_DIM_SCALE*dimensions[2])),
                btScalar(B_DIM_SCALE*dimensions[3]) );

            volume = 0.0;
            shape_string = collision_shape_->getName();

            break;
        }

        case SPHERE : {
            // check if we have all the dimensions
            if (dimensions.size() != 1)
                throw std::runtime_error("BulletVTKObject SPHERE shape requires "
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
            volume = 4/3*M_PI* pow(dimensions[0], 3);

            // set name
            shape_string = collision_shape_->getName();

            break;
        }
// -------------------------------------------------------------------------
        case CYLINDER : {
            // check if we have all the dimensions
            if (dimensions.size() != 2)
                throw std::runtime_error("BulletVTKObject CYLINDER shape requires "
                                                 "a vector of 2 doubles "
                                                 "as dimensions.");
            // VTK actor_
            vtkSmartPointer<vtkCylinderSource> source =
                    vtkSmartPointer<vtkCylinderSource>::New();

            source->SetRadius(dimensions[0]);
            source->SetHeight(dimensions[1]);
            source->SetResolution(30);
            mapper->SetInputConnection(source->GetOutputPort());

            // Bullet Shape
            collision_shape_ =
                    new btCylinderShape(
                        btVector3(btScalar(B_DIM_SCALE*dimensions[0]),
                                  btScalar(B_DIM_SCALE*dimensions[1]/2), 0.0
                    ));

            // calculate volume
            volume = M_PI * pow(dimensions[0], 2) * dimensions[1];

            // set name
            shape_string = collision_shape_->getName();

            break;
        }

        case BOX : {

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
            volume = dimensions[0] *
                    dimensions[1] *
                    dimensions[2];

            // set name
            shape_string = collision_shape_->getName();;

            break;
        }
        case CONE : {

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
                    btScalar(B_DIM_SCALE*dimensions[0]),
                    btScalar(B_DIM_SCALE*dimensions[1]/2));

            // calculate volume
            volume = float(M_PI* pow(dimensions[0], 2) *
                                   dimensions[1]/3);

            // set name
            shape_string = collision_shape_->getName();;

            break;
        }

        case MESH : {

            std::string* filepath = static_cast<std::string*>(data);
            if(!FileExists(filepath->c_str())) {
                ROS_ERROR("Can't open mesh file: %s", filepath->c_str());
                throw std::runtime_error("Can't open mesh file.");
            }
            else
                ROS_DEBUG("Loading mesh file from: %s", filepath->c_str()) ;

            collision_shape_ = LoadCompoundMeshFromObj(*filepath, B_DIM_SCALE);

            // set name
            shape_string = collision_shape_->getName();;

            // -------------------------------------------------------------------------
            // VTK TODO: We have already read the Mesh object, we should use
            // the vertices from that instead of reading it again with VTK
            // reader.

            vtkSmartPointer<vtkOBJReader> reader =
                vtkSmartPointer<vtkOBJReader>::New();
            //
            ////            // visualize the compund mesh for debug
            //size_t last_dot_position = filepath->find_last_of(".");
            //
            //std::string file_name_no_extension = filepath->substr(0,
            //                                                      last_dot_position);
            //
            //std::stringstream out_name;
            //out_name << file_name_no_extension << "_hacd.obj";
            //
            //reader->SetFileName(out_name.str().c_str());

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
            shape == ObjectShape::CYLINDER ||
            shape == ObjectShape::BOX
            ){
            body_->setRollingFriction(btScalar(0.001));
            body_->setSpinningFriction(btScalar(0.001));
            //            body_->setAnisotropicFriction
            //                    (collision_shape_->getAnisotropicRollingFrictionDirection
            //                            (),btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
        }
        else if(shape == ObjectShape::SPHERE) {
            body_->setRollingFriction(btScalar(0.001));
            body_->setSpinningFriction(btScalar(0.001));
        }

        body_->setFriction(btScalar(friction));
        body_->setSpinningFriction(btScalar(0.001));


        ////set contact parameters
        //body_->setContactStiffnessAndDamping(btScalar(10000),
        //                                     btScalar(0.1));

        std::stringstream debug_msg;

        debug_msg << std::string(" shape = ") << shape_string
                  << ", mass = " << bt_mass
                  << ", volume = " << volume
                  << ", friction = " << body_->getFriction()
                  << ", Rolling Friction = " << body_->getRollingFriction()
                  << ", Spinning Friction = " << body_->getSpinningFriction();
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


