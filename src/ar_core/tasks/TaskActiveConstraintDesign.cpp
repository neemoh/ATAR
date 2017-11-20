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
        mesh_pose = KDL::Frame(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.06, 0.06, 0.02 ));
        mesh_pose.M.DoRotZ(M_PI);
        mesh_pose.M.DoRotX(M_PI/2);

        // construct the object
        kidney = new SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                               RESOURCES_DIRECTORY+"/mesh/kidney.obj", mesh_pose);
        kidney->GetActor()->GetProperty()->SetColor(colors.RedDark);

        AddSimObjectToTask(kidney);
    }


    // cell locator stuff
    cellLocator = vtkSmartPointer<vtkCellLocator>::New();
    cellLocator->SetDataSet(kidney->GetActor()->GetMapper()->GetInput());
    cellLocator->BuildLocator();

    // -------------------------------------------------------------------------
    // Create 6 dynamic spheres
    {
        // define object dimensions
        std::vector<double> sphere_dimensions = {0.005};

        // construct the object. Sor Sphere and planes we can have a
        // texture image too!
        sphere_tool = new SimObject(ObjectShape::SPHERE, ObjectType::KINEMATIC,
                                  sphere_dimensions);
        // we need to add the SimObject to the task. Without this step
        // the object is not going to be used
        AddSimObjectToTask(sphere_tool);
    }



    // -------------------------------------------------------------------------
    // Define a master manipulator
    master = new Manipulator("sigma7",
                                "/sigma7/sigma0/pose",
                                "/sigma7/sigma0/gripper_angle");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(master);


}

void TaskActiveConstraintDesign::TaskLoop() {


    sphere_tool->SetKinematicPose(master->GetPoseWorld());

    // -------------------------------------------------------------------------
    // Finding closest point from tool to board


    KDL::Vector center_point_in_mesh_local = mesh_pose.Inverse() *sphere_tool->GetPose().p;

    double board_central_point_in_mesh_local[3] = {center_point_in_mesh_local[0],
                                                   center_point_in_mesh_local[1],
                                                   center_point_in_mesh_local[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};
    double closestPointDist2; //the squared distance to the closest point
    vtkIdType cell_id; //the cell id of the cell containing the closest point
    int subId;

    cellLocator->Update();
    cellLocator->FindClosestPoint(board_central_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);

    KDL::Vector closest_point_to_center_point = mesh_pose *
                                                KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    closest_point[0] = closest_point_to_center_point.data[0];
    closest_point[1] = closest_point_to_center_point.data[1];
    closest_point[2] = closest_point_to_center_point.data[2];

    //-----------------------------------------------------------------------
    // Drawing line from tool to closest point

    // Line initialized in .h to create a new one every loop
    vtkSmartPointer<vtkPolyDataMapper> closestLineMapper = vtkPolyDataMapper::New();
    vtkSmartPointer<vtkActor> closestActor = vtkActor::New();

    double currentPoint[3] = {sphere_tool->GetPose().p[0],
                              sphere_tool->GetPose().p[1],
                              sphere_tool->GetPose().p[2]};

    // define two points for the line to be drawn
    closestLine->SetPoint1(currentPoint);
    closestLine->SetPoint2(closest_point);
    closestLine->Update();

    closestLineMapper->SetInputData(closestLine->GetOutput());
    closestLineMapper->Update();

    // Define closest line properties
    closestActor->SetMapper(closestLineMapper);
    closestActor->GetProperty()->SetColor(colors.Red);
    closestActor->GetProperty()->SetLineWidth(2);

    // add closest point line to the graphics
    graphics->AddActorToScene(closestActor);

    //-----------------------------------------------------------------------
    // Draw trajectory line on board

    // Initialize new line
    line = vtkLineSource::New();
    lineMapper = vtkPolyDataMapper::New();
    vtkSmartPointer<vtkActor> actor = vtkActor::New();

    // If it's the first segment of trajectory then draw the first point...

    ROS_INFO("%f",master->GetGripper());
    if(master->GetGripper()<0.1) {
        if (start) {
            line->SetPoint1(closest_point);
            line->SetPoint2(closest_point);
            start = false;
        }
            //... else draw the trajectory
        else {
            line->SetPoint1(lastPoint);
            line->SetPoint2(closest_point);
        }
        line->Update();

        // update last point for trajectory
        lastPoint[0] = closest_point[0];
        lastPoint[1] = closest_point[1];
        lastPoint[2] = closest_point[2];

        lineMapper->SetInputData(line->GetOutput());
        lineMapper->Update();

        // Define trajectory line property
        actor->SetMapper(lineMapper);
        actor->GetProperty()->SetColor(colors.Gold);
        actor->GetProperty()->SetLineWidth(7);

        // Add actor of trajectory line to graphics
        graphics->AddActorToScene(actor);
    }
}

TaskActiveConstraintDesign::~TaskActiveConstraintDesign() {

    delete master;

}
