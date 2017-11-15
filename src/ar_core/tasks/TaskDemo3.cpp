//
// Created by nima on 14/11/17.
//

#include "TaskDemo3.h"

TaskDemo3::TaskDemo3()
{
    // the only needed argument to construct a Renderer is the nodehandle ptr
    // The rest have default values.
    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({920, 640}),
            /*ar_mode=*/true,
            /*n_views=*/1,
            /*one_window_per_view=*/false,
            /*borders_off=*/false,
            /*window_positions=*/std::vector<int>({700,50}));

    // -------------------------------------------------------------------------
    // Create 1 simple dynamic cube (pointer local in the constructor)
    SimObject *cube;
    {
        // define object dimensions
        std::vector<double> dimensions = {0.02, 0.01, 0.005};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.07, 0.055, 0.05 ));

        // construct the object
        cube = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC,
                             dimensions);

        // we can access all the properties of a VTK actor
        cube->GetActor()->GetProperty()->SetColor(colors.OrangeRed);

        // we need to add the SimObject to the task. Without this step
        // the object is not going to be used
        AddSimObjectToTask(cube);
    }

//    throw std::runtime_error("Yo");
}

void TaskDemo3::TaskLoop() {

}
