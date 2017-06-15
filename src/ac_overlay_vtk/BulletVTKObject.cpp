//
// Created by nima on 15/06/17.
//

#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <kdl/frames.hpp>
#include "BulletVTKObject.h"

uint BulletVTKObject::num_bulletvtk_objects = 0;


BulletVTKObject::BulletVTKObject(ObjectShape shape, ObjectType type,
                                 std::vector<double> dimensions,
                                 double pose[],
                                 double mass) {

    num_bulletvtk_objects++;
    std::cout << " num_bulletvtk_objects " << num_bulletvtk_objects <<
                                                                    std::endl;


    // common objects

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    actor = vtkSmartPointer<vtkActor>::New();
    //
    switch (shape){

        case ObjectShape::SPHERE : {
            // check if we have all dimensions
            if (dimensions.size() != 1)
                throw std::runtime_error("BulletVTKObject BOX shape requires "
                                                 "a vector of 1 double "
                                                 "as dimensions.");

            // VTK actor
            vtkSmartPointer<vtkSphereSource> source =
                    vtkSmartPointer<vtkSphereSource>::New();

            source->SetRadius(dimensions[0]);
            source->SetPhiResolution(30);
            source->SetThetaResolution(30);
            mapper->SetInputConnection(source->GetOutputPort());
            actor->SetMapper(mapper);

            // Bullet Shape
            collision_shape = new btSphereShape(btScalar(dimensions[0]));

            break;
        }

        case ObjectShape::BOX : {

            // check if we have all dimensions
            if (dimensions.size() != 3)
                throw std::runtime_error("BulletVTKObject BOX shape requires "
                                                 "a vector of three doubles "
                                                 "as dimensions.");

            // VTK actor
            vtkSmartPointer<vtkCubeSource> board_source =
                    vtkSmartPointer<vtkCubeSource>::New();

            board_source->SetXLength(dimensions[0]);
            board_source->SetYLength(dimensions[1]);
            board_source->SetZLength(dimensions[2]);

            mapper->SetInputConnection(board_source->GetOutputPort());
            actor->SetMapper(mapper);

            // Bullet Shape
            collision_shape = new btBoxShape(
                    btVector3(btScalar(dimensions[0]/2), btScalar
                                      (dimensions[1]/2),
                              btScalar(dimensions[2]/2)));

            break;
        }
        case ObjectShape::CONE : {

            // check if we have all dimensions
            if (dimensions.size() != 2)
                throw std::runtime_error("BulletVTKObject CONE shape requires "
                                                 "a vector of two doubles "
                                                 "as dimensions.");
            collision_shape = new btConeShape(
                    btScalar(dimensions[0]), btScalar(dimensions[1]));

            // TODO: ADD VTK PART
            break;
        }
    }

    //------------------------------------------------------------------------------
    // Set initial pose of graphical representation
    actor->SetUserMatrix(PoseVectorToVTKMatrix(pose));

    //------------------------------------------------------------------------------
    // set up dynamics
    // rigid body is dynamic if and only if mass is non zero, otherwise static
    btScalar bt_mass((float)mass);
    bool isDynamic = (bt_mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
        collision_shape->calculateLocalInertia(bt_mass, localInertia);

    btTransform init_transform;
    init_transform.setIdentity();
    init_transform.setOrigin(btVector3((float) pose[0], (float) pose[1],
                                       (float) pose[2]));
    init_transform.setRotation(btQuaternion((float) pose[3], (float) pose[4],
                                            (float) pose[5], (float) pose[6]));

    motion_state = new BulletVTKMotionState(init_transform, actor);

    btRigidBody::btRigidBodyConstructionInfo rb_info(
            mass, motion_state, collision_shape, localInertia);

    body = new btRigidBody(rb_info);

}


BulletVTKObject::~BulletVTKObject() {

    delete collision_shape;
//    delete motion_state;
//        delete body;
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
