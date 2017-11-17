//
// Created by nima on 14/11/17.
//

#include "TaskDemo3.h"

TaskDemo3::TaskDemo3()
{
    // create a rendering object with desired parameters. In particular, we
    // want to set the ar_mode flag to true
    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({920, 640}),
            /*ar_mode=*/true,
            /*n_views=*/1,
            /*one_window_per_view=*/false,
            /*borders_off=*/false,
            /*window_positions=*/std::vector<int>({700,50}));

    ros::NodeHandle n("~");
    // -------------------------------------------------------------------------
    // Create 4 cubes with no dynamics and put them on 4 squares of the
    // charuco board

    // get the length of the board squares
    std::vector<float> board_params = std::vector<float>(5, 0.0);
    n.getParam("board_params", board_params);
    double distance = board_params[3];

    // create the positions of the 4 cubes
    double positions[4][2] = {
            {distance/2,                distance/2 },
            {5*distance + distance/2,   distance/2 },
            {5*distance + distance/2,   3*distance +distance/2 },
            {distance/2,                3*distance +distance/2 },
      };

    SimObject *cube[4];

    for(int i = 0; i < 4; ++i) {

        // define object dimensions
        std::vector<double> dimensions = {distance, distance, 0.05};

        // define object Pose
        KDL::Frame pose(KDL::Rotation::Quaternion(0., 0., 0., 1.),
                        KDL::Vector(positions[i][0], positions[i][1], 0.0));

        // construct the object
        cube[i] = new SimObject(ObjectShape::BOX, ObjectType::NOPHYSICS,
                             dimensions,pose);

        // we can access all the properties of a VTK actor
        cube[i]->GetActor()->GetProperty()->SetColor(colors.Red);

        // we need to add the SimObject to the task. Without this step
        // the object is not going to be used
        AddSimObjectToTask(cube[i]);
    }

}
