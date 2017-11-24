//
// Created by nima on 20/11/17.
//

#include "TaskActiveConstraintDesign.h"

TaskActiveConstraintDesign::TaskActiveConstraintDesign()
{

    // create a rendering object with desired parameters
    graphics = std::make_unique<Rendering>(
            /*view_resolution=*/std::vector<int>({920, 640}),
            /*ar_mode=*/true,
            /*n_views=*/1,
            /*one_window_per_view=*/false,
            /*borders_off=*/false,
            /*window_positions=*/std::vector<int>({300,50}));

    auto temp_pose = graphics->GetMainCameraPose();
    temp_pose.M.DoRotX(-30./180.*M_PI);
    temp_pose.p = KDL::Vector(0.06, -0.05, 0.28);

    graphics->SetMainCameraPose(temp_pose);

    // -------------------------------------------------------------------------
    // Define a master manipulator
    master = new Manipulator("sigma",
                             "/sigma/sigma0/dummy_slave_pose",
                             "/sigma/sigma0/gripper_angle",
                             "/sigma/sigma0/buttons");

    // for correct calibration, the master needs the pose of the camera
    graphics->SetManipulatorInterestedInCamPose(master);


    // -------------------------------------------------------------------------
    // static floor
    // always add a floor under the workspace of your task to prevent objects
    // from falling too far and mess things up.
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE, floor_dims);
    AddSimObjectToTask(floor);

    // -------------------------------------------------------------------------
//    // create a floor
//    SimObject* plane;
//    plane = new SimObject(ObjectShape::PLANE, ObjectType::DYNAMIC,
//                          std::vector<double>({0.15,0.10}),
//                          KDL::Frame(KDL::Vector(0.075, 0.05, 0.001)));
//    // add to simulation
//    AddSimObjectToTask(plane);
    // -------------------------------------------------------------------------
    // Create a mesh object
    SimObject *kidney;
    {
        // define object Pose
        mesh_pose = KDL::Frame(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector(0.15, 0.12, 0.02 ));
        mesh_pose.M.DoRotZ(M_PI);
        mesh_pose.M.DoRotX(M_PI/2);

        // construct the object
        kidney = new SimObject(ObjectShape::MESH, ObjectType::NOPHYSICS,
                               RESOURCES_DIRECTORY+"/mesh/kidney_wire.obj",
                               mesh_pose);
        kidney->GetActor()->GetProperty()->SetColor(colors.BlueDodger);

        AddSimObjectToTask(kidney);
    }

    // cell locator stuff
    cellLocator = vtkSmartPointer<vtkCellLocator>::New();
    cellLocator->SetDataSet(kidney->GetActor()->GetMapper()->GetInput());
    cellLocator->BuildLocator();

    // -------------------------------------------------------------------------
    // Create a sphere as tool
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


    //-----------------------------------------------------------------------
    // Drawing line from tool to the closest point
    vtkSmartPointer<vtkPolyDataMapper> closest_line_mapper =
            vtkPolyDataMapper::New();
    vtkSmartPointer<vtkActor> closest_actor = vtkActor::New();

    closest_line_mapper->SetInputData(closestLine->GetOutput());
    closest_line_mapper->Update();

    // Define closest line properties
    closest_actor->SetMapper(closest_line_mapper);
    closest_actor->GetProperty()->SetColor(colors.BlueDodger);
    closest_actor->GetProperty()->SetLineWidth(2);

    // add closest point line to the graphics
    graphics->AddActorToScene(closest_actor);


    //-----------------------------------------------------------------------
    // path
    path.GetActor()->GetProperty()->SetColor(colors.Gold);
    path.GetActor()->GetProperty()->SetLineWidth(7);

    // Add actor of trajectory line to graphics
    graphics->AddActorToScene(path.GetActor());
}

//------------------------------------------------------------------------------
void TaskActiveConstraintDesign::TaskLoop() {

    // Update virtual tool pose
    KDL::Frame master_pose = master->GetPoseWorld();
    sphere_tool->SetKinematicPose(master_pose);

    // get the buttons of the master
    int buttons[2];
    master->GetButtons(buttons);

    // rotate the camera
    auto rotate_cam_now = (bool)buttons[1];
    if(rotate_cam_now){
        if(first_iteration_cam_move) {
            master_initial_pose_for_cam_move =  master->GetPoseWorld();;
            camera_initial_pose_for_cam_move = graphics->GetMainCameraPose();
            first_iteration_cam_move = false;
        }
        else {
            KDL::Frame delta_master;
            delta_master.p =  master->GetPoseWorld().p -
                    master_initial_pose_for_cam_move.p;
            KDL::Frame cam_pose;
            KDL::Rotation temp;
            temp.DoRotX(-M_PI_2);
            delta_master.p = temp * cam_pose.M *delta_master.p;

            cam_pose.p = camera_initial_pose_for_cam_move.p + delta_master.p;
//            cam_pose.M = delta_master.M.Inverse() *
//                    camera_initial_pose_for_cam_move.M * delta_master.M;
            cam_pose.M = camera_initial_pose_for_cam_move.M;
            graphics->SetMainCameraPose(cam_pose);
        }
    } else
        first_iteration_cam_move = true;



    // Finding closest point from tool to mesh
    double closest_point[3] = {0.0, 0.0, 0.0};
    FindClosestPointToMesh(closest_point);

    // Update the closest point line
    double current_point[3] = {master_pose.p[0],
                               master_pose.p[1], master_pose.p[2]};
    closestLine->SetPoint1(current_point);
    closestLine->SetPoint2(closest_point);
    closestLine->Update();

    // Draw the path line on mesh
    bool draw_now = master->GetGripperAngles()<0.01;

    if(draw_now)
        path.InsertNewPoint(KDL::Vector(closest_point[0],
                                        closest_point[1],closest_point[2]));

}

//------------------------------------------------------------------------------
TaskActiveConstraintDesign::~TaskActiveConstraintDesign() {

    delete master;

}

//------------------------------------------------------------------------------
void TaskActiveConstraintDesign::FindClosestPointToMesh(double * out) {

    KDL::Vector tool_pos_in_mesh_frame = mesh_pose.Inverse() *sphere_tool->GetPose().p;

    double board_central_point_in_mesh_local[3] = {tool_pos_in_mesh_frame[0],
                                                   tool_pos_in_mesh_frame[1],
                                                   tool_pos_in_mesh_frame[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};
    double closestPointDist2; //the squared distance to the closest point
    vtkIdType cell_id; //the cell id of the cell containing the closest point
    int subId;

    cellLocator->Update();
    cellLocator->FindClosestPoint(board_central_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);

    KDL::Vector closest_point_in_world_frame = mesh_pose *
                                               KDL::Vector(closest_point[0],
                                                           closest_point[1],
                                                           closest_point[2]);

    out[0] = closest_point_in_world_frame.data[0];
    out[1] = closest_point_in_world_frame.data[1];
    out[2] = closest_point_in_world_frame.data[2];
}


