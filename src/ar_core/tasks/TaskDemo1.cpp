//
// Created by nima on 13/06/17.
//

#include "TaskDemo1.h"
#include <custom_conversions/Conversions.h>

TaskDemo1::TaskDemo1()
{

    // Initialize the graphics with default values. Check Demo2 to see more
    // details about the parameters you can pass to the Rendering class.
    graphics = std::make_unique<Rendering>();

    // CREATE SIMULATED OBJECTS
    // -------------------------------------------------------------------------
    // Create a board
    SimObject *board;
    {
        // define object dimensions
        std::vector<double> board_dimensions = {0.15, 0.10, 0.01};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.05, 0.05, -0.05) );

        // we want a static floor so ObjectType is DYNAMIC and no density is
        // passed (default is zero)
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                              board_dimensions, pose);

        // we can access all the properties of a VTK actor
        board->GetActor()->GetProperty()->SetColor(colors.Gray);

        // we need to add the rigid body to the dynamics workd and the actor
        // to the graphics_actors vector
        AddSimObjectToTask(board);
    }

    // -------------------------------------------------------------------------
    // Create 1 simple dynamic cube (pointer local in the constructor)
    SimObject *cube;
    {
        // define object dimensions
        std::vector<double> dimensions = {0.02, 0.01, 0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.07, 0.055, 0.05 ));

        // Define density
        double density = 50000; // kg/m3

        // construct the object
        cube = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                             dimensions, pose, density);

        // we can access all the properties of a VTK actor
        cube->GetActor()->GetProperty()->SetColor(colors.OrangeRed);

        // we need to add the SimObject to the task. Without this step
        // the object is not going to be used
        AddSimObjectToTask(cube);
    }

    // -------------------------------------------------------------------------
    // Create 6 dynamic mesh objects
    SimObject *monkey;
    {
        // define object dimensions
        std::vector<double> dimensions = {0.01, 0.01, 0.01};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.05, 0.055, 0.05 ));

        // Define
        double density = 50000; // kg/m3
        double friction = 0.4;

        for (int i = 0; i < 5; ++i) {

            // increment the pose of each object
            pose.p = pose.p+ KDL::Vector(0.001, 0.0, 0.03);

            // construct the object
            monkey = new SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                                 RESOURCES_DIRECTORY+"/mesh/monkey.obj", pose,
                                   density,friction);
            monkey->GetActor()->GetProperty()->SetColor(colors.OrangeDark);

            AddSimObjectToTask(monkey);
        }
    }

    // -------------------------------------------------------------------------
    // Create 6 dynamic spheres
    {
        // define object dimensions
        std::vector<double> sphere_dimensions = {0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.02, 0.05, 0.05 ));

        // Define
        double density = 50000; // kg/m3
        //override the default friction value
        float friction = 0.2;
        for (int i = 0; i < 6; ++i) {

            // increment the pose of each object
            pose.p = pose.p+ KDL::Vector(0.0001, 0.0, 0.008);

            // construct the object. Sor Sphere and planes we can have a
            // texture image too!
            sphere[i] = new SimObject(ObjectShape::SPHERE, ObjectType::DYNAMIC,
                                      sphere_dimensions, pose, density, friction,
                                      RESOURCES_DIRECTORY+"/texture/clouds.jpg");

            // we can access all the properties of a VTK actor
            // same applies for bullet parameters
            sphere[i]->GetBody()->setFriction(1.5f);
            sphere[i]->GetBody()->setRollingFriction(0.5f);
            sphere[i]->GetBody()->setSpinningFriction(0.5f);

            // we need to add the SimObject to the task. Without this step
            // the object is not going to be used
            AddSimObjectToTask(sphere[i]);
        }
    }

    // -------------------------------------------------------------------------
    // static floor
    // always add a floor under the workspace of your task to prevent objects
    // from falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE, floor_dims);
    AddSimObjectToTask(floor);

};

//------------------------------------------------------------------------------
void TaskDemo1::TaskLoop() {

    // let's get the pose of the camera and incrementally rotate it for fun!
    KDL::Frame cam_p = graphics->GetMainCameraPose();
    cam_p.M.DoRotZ(-0.005);
    cam_p.p.y(-0.05);
    cam_p.p.z(0.25);
    graphics->SetMainCameraPose(cam_p);

    // FYI you can access the pose of the objects:
    //    ROS_INFO("Sphere0 z: %f",sphere[0]->GetPose().p[2]);
}





