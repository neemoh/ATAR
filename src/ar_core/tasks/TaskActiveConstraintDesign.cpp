//
// Created by nima on 20/11/17.
//

#include "TaskActiveConstraintDesign.h"

TaskActiveConstraintDesign::TaskActiveConstraintDesign()
{

    // create a rendering object with desired parameters
    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({920, 640}),
            /*ar_mode=*/false,
            /*n_views=*/1,
            /*one_window_per_view=*/false,
            /*borders_off=*/false,
            /*window_positions=*/std::vector<int>({300,50}));

    auto temp_pose = graphics->GetMainCameraPose();
    temp_pose.M.DoRotX(-30./180.*M_PI);
    temp_pose.p = KDL::Vector(0.06, -0.05, 0.28);

    graphics->SetMainCameraPose(temp_pose);
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor under the workspace of your task to prevent objects
    // from falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE, floor_dims);
    AddSimObjectToTask(floor);

    // -------------------------------------------------------------------------
    // create a floor
    SimObject* plane;
    plane = new SimObject(ObjectShape::PLANE, ObjectType::DYNAMIC,
                          std::vector<double>({0.15,0.10}),
                          KDL::Frame(KDL::Vector(0.075, 0.05, 0.001)));
    // add to simulation
    AddSimObjectToTask(plane);
    // -------------------------------------------------------------------------
    // Create a mesh object
    SimObject *kidney;
    {
        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.06, 0.06, 0.02 ));
        pose.M.DoRotZ(M_PI);
        pose.M.DoRotX(M_PI/2);

        // construct the object
        kidney = new SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                               RESOURCES_DIRECTORY+"/mesh/kidney.obj", pose);
        kidney->GetActor()->GetProperty()->SetColor(colors.RedDark);

        AddSimObjectToTask(kidney);
    }
}

void TaskActiveConstraintDesign::TaskLoop() {

}
