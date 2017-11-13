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
 *
 * When a SimObject is generated its graphic actor (actor_) and physics body
 * (rigid_body_) has to be added to the renderer and physics world.
 * After which, the pose of the graphical representation will be
 * automatically updated by the physics simulation, thanks the
 * toBulletVTKMotionState member. To learn more about how to add simObjects
 * to the simulation world refer to VTKTask.h
 *
 * \arg To construct a SimObject at least 3 arguments are needed:
 *
 *      ObjectShape: Can be predefined shapes (ex: ObjectType::SPHERE) or mesh
 *      (ObjectType::MESH). In the Mesh case the address of the .obj file
 *      must be provided with the mesh_address argument.
 *
 *      dimensions: If ObjectShape is one of the predefined shapes the
 *      dimensions must be specified as an array described in ObjectShape
 *      enum definition.
 *
 *      ObjectType: Four cases
 *          1. NOPHYSICS: The physics will not be included.
 *          2. KINEMATIC: The pose of the object will be set at each
 *          iteration using the SetKinematicPose. This is useful for things
 *          like tools that their pose is read from a master device.
 *          3. DYNAMIC and density!=0.0 The pose of the object will be
 *          automatically update from the physics simulation.
 *          4. DYNAMIC and no density provided (density defults to 0.0) in
 *          this case the object will be Static, meaning its pose will be
 *          constant but dynamic objects can collide with the object
 * Other params:
 *      Density: Is needed for dynamic objects to calculate their mass.
 *      pose: The Initial pose of the dynamics objects.
 *      friction: friction!
 *      id: can be used in collision detection
 *      texture_address: For Sphere and Plane shapes you can have a texture
 *      image. PNG and JPG formats are supported.
 *
 * \attention MESH:  Meshes are decomposed into approximated compound meshes
 * using the VHACD method. Generating the approximated compound mesh can take
 * up to a few minutes. So we do this only once and save the compound mesh in
 * a separate file that has the same name of the original mesh file with an
 * added _hacd. Next time the application is executed we search for the file
 * with _hacd and if found, it is used and compound mesh generation is not
 * repeated.
 * Note: The generated compound meshes are approximate and
 * sometimes the approximation deviates considerably from the original mesh.
 * To check how the generated compound object looks like, you can either open
 * the generated <filename>_hacd.obj in blender or set the show_compound_mesh
 * boolean to true in the constructor of SimObject.
 *
 * \attention Dimension scaling: We were interested in objects with
 * dimensions in the order of a few millimiters. It turned out that the bullet
 * simulation becomes unstable for such small dimensions. To get around this,
 * the dimensions of all the bullet related things are multiplied by
 * B_DIM_SCALE.
 *
 * \author Nima Enayati
 * \date June 2017
 */




// -----------------------------------------------------------------------------
enum ObjectType {
    NOPHYSICS,  // Just graphics
    NOVISUALS,  // Just graphics
    DYNAMIC,    // If density==0.0 object is STATIC
    KINEMATIC,   // Set the pose of the object externally
};

// -----------------------------------------------------------------------------
enum ObjectShape {
    STATICPLANE,    //dims = [normal_x, normal_y, normal_z, constant_distance]
    PLANE,          //dims =
    SPHERE,         //dims = [radius]
    CYLINDER,       //dims = [radius, height]
    BOX,            //dims = [width, length, height]
    CONE,           //dims = [radius, height]
    MESH
};

// -----------------------------------------------------------------------------
class SimObject {

public:
    /**
    * SimObject target constructor. Needs at least 3 arguments.
    */
    SimObject(ObjectShape shape,
              ObjectType type,
              std::vector<double> dimensions,
              const KDL::Frame &pose=KDL::Frame(),
              double density = 0.0,
              double friction = 0.1,
              std::string texture_address = {},
              std::string mesh_address = {},
              int id = 0);

    // Delegated constructor for mesh objects doesnt have dimensions
    SimObject(ObjectShape shape,
              const ObjectType type,
              std::string mesh_address,
              const KDL::Frame &pose=KDL::Frame(),
              double density = 0.0,
              double friction = 0.1,
              int id = 0)
            :
            SimObject::SimObject(shape, type, std::vector<double>(),
                                 pose, density, friction, "",
                                 std::move(mesh_address), id){};

    // Delegated constructor for staticplane
    SimObject(ObjectShape shape,std::vector<double> dimensions)
            :
            SimObject(shape, DYNAMIC, std::move(dimensions)) {
        if(shape!=STATICPLANE)
            throw std::runtime_error("SimObject constructor with two argument "
                                             "is reserved only for STATICPLANE"
                                             " shape");}

    ~SimObject();

    /**
    * Returns the rigid body member of the object.
    */
    btRigidBody* GetBody() { return rigid_body_; }

    /**
    * Returns the VTK actor member of the object.
    */
    vtkSmartPointer<vtkActor> GetActor() { return actor_; };

    /**
    * Set the pose if the object is kinematic.
    */
    void SetKinematicPose(const KDL::Frame & pose);

    /**
    * Can be used in physics collision handling
    */
    int GetId() {return id_; };

    /**
    * Get the pose of the object.
    */
    KDL::Frame GetPose();

    ObjectType GetObjectType(){return object_type_;}

private:

    int id_;
    ObjectType                   object_type_;
    btRigidBody *                rigid_body_;
    vtkSmartPointer<vtkActor>    actor_;
    BulletVTKMotionState  *      motion_state_;
    btCollisionShape *           collision_shape_;

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
/**
* callback for pairwise collision test.
*/
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
