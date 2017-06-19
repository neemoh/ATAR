//
// Created by nima on 15/06/17.
//

#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <kdl/frames.hpp>
#include <vtkConeSource.h>
#include "BulletVTKObject.h"

uint BulletVTKObject::num_bulletvtk_objects = 0;


BulletVTKObject::BulletVTKObject(ObjectShape shape, ObjectType o_type,
                                 std::vector<double> dimensions,
                                 double pose[],
                                 double density,
                                 double contact_stiffness,
                                 double contact_damping,
                                 double friction,
                                 double restitution)
        : object_type_(o_type)
{

    num_bulletvtk_objects++;
    std::cout << " num_bulletvtk_objects " << num_bulletvtk_objects <<
              std::endl;

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
            collision_shape_ = new btSphereShape(btScalar(dimensions[0]));

            // calculate volume
            volume = 4/3*M_PI* pow(dimensions[0], 3);

            // set name
            shape_string = "SPHERE";

            break;
        }

        case ObjectShape::BOX : {

            // check if we have all dimensions
            if (dimensions.size() != 3)
                throw std::runtime_error("BulletVTKObject BOX shape requires "
                                                 "a vector of three doubles "
                                                 "as dimensions.");
            // calculate volume
            volume = dimensions[0] * dimensions[1] * dimensions[2];


            // VTK actor_
            vtkSmartPointer<vtkCubeSource> board_source =
                    vtkSmartPointer<vtkCubeSource>::New();

            board_source->SetXLength(dimensions[0]);
            board_source->SetYLength(dimensions[1]);
            board_source->SetZLength(dimensions[2]);

            mapper->SetInputConnection(board_source->GetOutputPort());

            // Bullet Shape
            collision_shape_ = new btBoxShape(
                    btVector3(btScalar(dimensions[0]/2),
                              btScalar(dimensions[1]/2),
                              btScalar(dimensions[2]/2)));
            // set name
            shape_string = "BOX";

            break;
        }
        case ObjectShape::CONE : {

            // check if we have all dimensions
            if (dimensions.size() != 2)
                throw std::runtime_error("BulletVTKObject CONE shape requires "
                                                 "a vector of two doubles "
                                                 "as dimensions.");

            // VTK actor_
            vtkSmartPointer<vtkConeSource> cone_source =
                    vtkSmartPointer<vtkConeSource>::New();

            cone_source->SetRadius(dimensions[0]);
            cone_source->SetHeight(dimensions[1]);

            mapper->SetInputConnection(cone_source->GetOutputPort());

            // Bullet Shape
            collision_shape_ = new btConeShape(
                    btScalar(dimensions[0]), btScalar(dimensions[1]));

            // calculate volume
            volume = float(M_PI* pow(dimensions[0], 2) * dimensions[1]/3);

            // set name
            shape_string = "CONE";

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

        // construct body_ info f
        btRigidBody::btRigidBodyConstructionInfo body_info(
                bt_mass, motion_state_, collision_shape_, local_inertia);
        body_info.m_restitution = (btScalar) restitution;
        body_info.m_friction = (btScalar) friction;

        // to prevent rounded objects from rolling for ever we add a bit of
        // rolling friction
        if (shape == ObjectShape::CONE || shape == ObjectShape::SPHERE)
            body_info.m_rollingFriction = 0.2;
        body_ = new btRigidBody(body_info);

        // set appropriate flags if the body is kinematic
        if (object_type_ == ObjectType::KINEMATIC) {
            body_->setCollisionFlags(body_->getCollisionFlags() |
                                     btCollisionObject::CF_KINEMATIC_OBJECT);
            body_->setActivationState(DISABLE_DEACTIVATION);
        }

        //set contact parameters
        body_->setContactStiffnessAndDamping((float) contact_stiffness,
                                             (float) contact_damping);

        std::cout << "Created BulletVTKObject with properties: "
                  << " shape = " << shape_string
                  << ", mass = " << bt_mass
                  << ", volume = " << volume
                  << ", restitution = " << restitution
                  << ", friction = " << friction
                  << ", contact_stiffness = " << contact_stiffness
                  << ", contact_damping = " << contact_damping
                  << std::endl;
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
        transform.setOrigin(btVector3((float) pose[0], (float) pose[1],
                                      (float) pose[2]));
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
