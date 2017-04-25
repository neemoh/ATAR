//
// Created by nima on 4/18/17.
//

#include <utils/Conversions.hpp>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include "BuzzWireTask.h"


namespace Colors {
    double Red[3] {1.0, 0.1, 0.03};
    double Green[3] {0.0, 0.9, 0.03};
    double Pink[3] {1.0, 0.0, 1.0};
    double Orange[3] {0.9, 0.4, 0.1};
    double Gray [3] {0.4, 0.4, 0.4};
    double Turquoise[3]	{0.25, 0.88, 0.82};
    double DeepPink[3] {1.0, 0.08, 0.58};
};

BuzzWireTask::BuzzWireTask(const double ring_radius,
                           const bool show_ref_frames)
        :
        ring_radius_(ring_radius),
        show_ref_frames(show_ref_frames),
        destination_cone_counter(0),
        ac_params_changed(true),
        task_state(TaskState::Idle),
        position_error_norm(0.0),
        number_of_repetition(0),
        idle_point(KDL::Vector(0.010, 0.011, 0.033)),
        start_point(KDL::Vector(0.017, 0.015, 0.033)),
        end_point(KDL::Vector(0.049, 0.028, 0.056)) {


    // -------------------------------------------------------------------------
    //  ACTIVE CONSTRAINT
    // -------------------------------------------------------------------------
    // these parameters could be set as ros parameters too but since
    // they change during the task I am hard coding them here.
    ac_parameters.method = 0; // 0 for visco/elastic
    ac_parameters.active = 0;

    ac_parameters.max_force = 4.0;
    ac_parameters.linear_elastic_coeff = 1000.0;
    ac_parameters.linear_damping_coeff = 10.0;

    ac_parameters.max_torque = 0.03;
    ac_parameters.angular_elastic_coeff = 0.04;
    ac_parameters.angular_damping_coeff = 0.002;


    // -------------------------------------------------------------------------
    //  INITIALIZING GRAPHICS ACTORS
    // -------------------------------------------------------------------------
    ring_actor = vtkSmartPointer<vtkActor>::New();
    error_sphere_actor = vtkSmartPointer<vtkActor>::New();

    tool_current_frame_axes = vtkSmartPointer<vtkAxesActor>::New();

    tool_desired_frame_axes = vtkSmartPointer<vtkAxesActor>::New();

    cellLocator = vtkSmartPointer<vtkCellLocator>::New();

    line1_source = vtkSmartPointer<vtkLineSource>::New();

    line2_source = vtkSmartPointer<vtkLineSource>::New();

    tool_current_pose = vtkSmartPointer<vtkMatrix4x4>::New();

    // -------------------------------------------------------------------------
    // RING
    double ring_cross_section_radius = 0.0005;
    double source_scales = 0.006;

    vtkSmartPointer<vtkParametricTorus> parametricObject =
            vtkSmartPointer<vtkParametricTorus>::New();
    parametricObject->SetCrossSectionRadius(
            ring_cross_section_radius / source_scales);
    parametricObject->SetRingRadius(ring_radius_ / source_scales);

    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
            vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();

    // to transform the data
    vtkSmartPointer<vtkTransformPolyDataFilter> ring_local_transform_filter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_local_transform_filter->SetInputConnection(
            parametricFunctionSource->GetOutputPort());

    vtkSmartPointer<vtkTransform> ring_local_transform =
            vtkSmartPointer<vtkTransform>::New();
    ring_local_transform->RotateX(90);
    ring_local_transform->Translate(0.0, ring_radius_ / source_scales, 0.0);

    ring_local_transform_filter->SetTransform(ring_local_transform);
    ring_local_transform_filter->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_mapper->SetInputConnection(
            ring_local_transform_filter->GetOutputPort());

    ring_actor->SetMapper(ring_mapper);
    ring_actor->SetScale(source_scales);
    ring_actor->GetProperty()->SetColor(Colors::Turquoise);
    ring_actor->GetProperty()->SetSpecular(0.7);

    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    tool_current_frame_axes->SetXAxisLabelText("");
    tool_current_frame_axes->SetYAxisLabelText("");
    tool_current_frame_axes->SetZAxisLabelText("");
    tool_current_frame_axes->SetTotalLength(0.007, 0.007, 0.007);
    tool_current_frame_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    tool_desired_frame_axes->SetXAxisLabelText("");
    tool_desired_frame_axes->SetYAxisLabelText("");
    tool_desired_frame_axes->SetZAxisLabelText("");
    tool_desired_frame_axes->SetTotalLength(0.007, 0.007, 0.007);
    tool_desired_frame_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);


    // -------------------------------------------------------------------------
    // Stand MESH hq
    std::string inputFilename = "/home/charm/Desktop/cads/task1_4_stand.STL";
    vtkSmartPointer<vtkSTLReader> stand_mesh_reader =
            vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    stand_mesh_reader->SetFileName(inputFilename.c_str());
    stand_mesh_reader->Update();

    // transform
    vtkSmartPointer<vtkTransform> stand_transform =
            vtkSmartPointer<vtkTransform>::New();
    stand_transform->Translate(0.065, 0.045, 0.025);
    stand_transform->RotateX(180);
    stand_transform->RotateZ(150);

    vtkSmartPointer<vtkTransformPolyDataFilter> stand_mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    stand_mesh_transformFilter->SetInputConnection(
            stand_mesh_reader->GetOutputPort());
    stand_mesh_transformFilter->SetTransform(stand_transform);
    stand_mesh_transformFilter->Update();


    vtkSmartPointer<vtkPolyDataMapper> stand_mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    stand_mesh_mapper->SetInputConnection(
            stand_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> stand_mesh_actor =
            vtkSmartPointer<vtkActor>::New();
    stand_mesh_actor->SetMapper(stand_mesh_mapper);
    stand_mesh_actor->GetProperty()->SetColor(Colors::Gray);
    //    stand_mesh_actor->GetProperty()->SetSpecular(0.8);


    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for finding generating
    // active constraints
    inputFilename = "/home/charm/Desktop/cads/task1_4_tube.STL";
    vtkSmartPointer<vtkSTLReader> hq_mesh_reader =
            vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    hq_mesh_reader->SetFileName(inputFilename.c_str());
    hq_mesh_reader->Update();
    vtkSmartPointer<vtkTransform> tube_transform =
            vtkSmartPointer<vtkTransform>::New();
    tube_transform->DeepCopy(stand_transform);
    tube_transform->RotateX(-15);

    vtkSmartPointer<vtkTransformPolyDataFilter> hq_mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    hq_mesh_transformFilter->SetInputConnection(
            hq_mesh_reader->GetOutputPort());
    hq_mesh_transformFilter->SetTransform(tube_transform);
    hq_mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> hq_mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    hq_mesh_mapper->SetInputConnection(
            hq_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> hq_mesh_actor = vtkSmartPointer<vtkActor>::New();
    hq_mesh_actor->SetMapper(hq_mesh_mapper);
    hq_mesh_actor->GetProperty()->SetColor(Colors::Orange);
    hq_mesh_actor->GetProperty()->SetSpecular(0.8);
    hq_mesh_actor->GetProperty()->SetSpecularPower(80);
    //    hq_mesh_actor->GetProperty()->SetOpacity(0.5);


    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for finding generating
    // active constraints
    inputFilename = "/home/charm/Desktop/cads/super_ring_lq.STL";
    vtkSmartPointer<vtkSTLReader> ring_guides_mesh_reader =
            vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    ring_guides_mesh_reader->SetFileName(inputFilename.c_str());
    ring_guides_mesh_reader->Update();

    // transform
    vtkSmartPointer<vtkTransform> ring_transform =
            vtkSmartPointer<vtkTransform>::New();
    ring_transform->Translate(0.0, 0.0, 0.0035);
    ring_transform->RotateX(90);

    vtkSmartPointer<vtkTransformPolyDataFilter> ring_guides_mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_guides_mesh_transformFilter->SetInputConnection(
            ring_guides_mesh_reader->GetOutputPort());
    ring_guides_mesh_transformFilter->SetTransform(ring_transform);
    ring_guides_mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_guides_mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_guides_mesh_mapper->SetInputConnection(
            ring_guides_mesh_transformFilter->GetOutputPort());

    ring_guides_mesh_actor = vtkSmartPointer<vtkActor>::New();
    ring_guides_mesh_actor->SetMapper(ring_guides_mesh_mapper);
    ring_guides_mesh_actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    ring_guides_mesh_actor->GetProperty()->SetOpacity(0.5);
    //    hq_mesh_actor->GetProperty()->SetOpacity(0.5);

    // -------------------------------------------------------------------------
    // MESH lq
    inputFilename = "/home/charm/Desktop/cads/task1_4_wire.STL";
    vtkSmartPointer<vtkSTLReader> lq_mesh_reader =
            vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    lq_mesh_reader->SetFileName(inputFilename.c_str());
    lq_mesh_reader->Update();


    vtkSmartPointer<vtkTransformPolyDataFilter> lq_mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    lq_mesh_transformFilter->SetInputConnection(
            lq_mesh_reader->GetOutputPort());
    lq_mesh_transformFilter->SetTransform(tube_transform);
    lq_mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> lq_mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    lq_mesh_mapper->SetInputConnection(
            lq_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> lq_mesh_actor = vtkSmartPointer<vtkActor>::New();
    lq_mesh_actor->SetMapper(lq_mesh_mapper);

    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(lq_mesh_transformFilter->GetOutput());
    cellLocator->BuildLocator();

    // -------------------------------------------------------------------------
    // Create a cube for the floor
    vtkSmartPointer<vtkCubeSource> floor_source =
            vtkSmartPointer<vtkCubeSource>::New();
    double floor_dimensions[3] = {0.1, 0.07, 0.001};
    floor_source->SetXLength(floor_dimensions[0]);
    floor_source->SetYLength(floor_dimensions[1]);
    floor_source->SetZLength(floor_dimensions[2]);
    // Create a sphere_mapper and actor.
    vtkSmartPointer<vtkPolyDataMapper> floor_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    floor_mapper->SetInputConnection(floor_source->GetOutputPort());
    vtkSmartPointer<vtkActor> floor_actor = vtkSmartPointer<vtkActor>::New();
    floor_actor->SetMapper(floor_mapper);
    floor_actor->SetPosition(floor_dimensions[0] / 2, floor_dimensions[1] / 2,
                             -floor_dimensions[2]);
    floor_actor->GetProperty()->SetOpacity(0.3);
    floor_actor->GetProperty()->SetColor(Colors::Pink);

    // -------------------------------------------------------------------------
    // Lines
    vtkSmartPointer<vtkPolyDataMapper> line1_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    vtkSmartPointer<vtkActor> line1_actor =
            vtkSmartPointer<vtkActor>::New();
    line1_actor->SetMapper(line1_mapper);
    line1_actor->GetProperty()->SetLineWidth(5);
    line1_actor->GetProperty()->SetColor(1, 0, 1);

    vtkSmartPointer<vtkPolyDataMapper> line2_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    line2_mapper->SetInputConnection(line2_source->GetOutputPort());
    vtkSmartPointer<vtkActor> line2_actor =
            vtkSmartPointer<vtkActor>::New();
    line2_actor->SetMapper(line2_mapper);
    line2_actor->GetProperty()->SetLineWidth(3);

    // -------------------------------------------------------------------------
    // destination cone
    vtkSmartPointer<vtkConeSource> destination_cone_source =
            vtkSmartPointer<vtkConeSource>::New();
    destination_cone_source->SetRadius(0.002 / source_scales);
    destination_cone_source->SetHeight(0.006 / source_scales);
    destination_cone_source->SetResolution(12);

    vtkSmartPointer<vtkPolyDataMapper> destination_cone_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    destination_cone_mapper->SetInputConnection(
            destination_cone_source->GetOutputPort());
    destination_cone_actor = vtkSmartPointer<vtkActor>::New();
    destination_cone_actor->SetMapper(destination_cone_mapper);
    destination_cone_actor->SetScale(source_scales);
    destination_cone_actor->GetProperty()->SetColor(Colors::Green);
    destination_cone_actor->GetProperty()->SetOpacity(0.5);
    destination_cone_actor->RotateY(90);
    destination_cone_actor->RotateZ(30);

    destination_cone_actor->SetPosition(idle_point[0], idle_point[1],
                                        idle_point[2]);


    //    cornerAnnotation =
    //            vtkSmartPointer<vtkCornerAnnotation>::New();
    //    cornerAnnotation->SetLinearFontScaleFactor( 2 );
    //    cornerAnnotation->SetNonlinearFontScaleFactor( 1 );
    //    cornerAnnotation->SetMaximumFontSize( 20 );
    //    cornerAnnotation->SetText( 0, "lower left" );
    //    cornerAnnotation->SetText( 1, "lower right" );
    //    cornerAnnotation->SetText( 2, "upper left" );
    //    cornerAnnotation->SetText( 3, "upper right" );
    ////    cornerAnnotation->GetTextProperty()->SetColor( 1, 0, 0 );



    // -------------------------------------------------------------------------
    // Error sphere
    vtkSmartPointer<vtkSphereSource> sphereSource =
            vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(-0.02, 0.08, 0.01);
    sphereSource->SetRadius(0.005);
    sphereSource->SetPhiResolution(15);
    sphereSource->SetThetaResolution(15);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(sphereSource->GetOutputPort());
    error_sphere_actor->SetMapper(sphere_mapper);
    error_sphere_actor->GetProperty()->SetColor(0.0, 0.7, 0.1);

    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);
        actors.push_back(tool_current_frame_axes);
        actors.push_back(tool_desired_frame_axes);
    }

    actors.push_back(hq_mesh_actor);
    actors.push_back(stand_mesh_actor);
    actors.push_back(lq_mesh_actor);
    //    actors.push_back(floor_actor);
    actors.push_back(ring_actor);
    actors.push_back(destination_cone_actor);
    //    actors.push_back(line1_actor);
    //    actors.push_back(line2_actor);
    actors.push_back(ring_guides_mesh_actor);
    actors.push_back(error_sphere_actor);
    // add the annotation as the last actor
    //    actors.push_back(cornerAnnotation);


}

//------------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkProp> > BuzzWireTask::GetActors() {
    return actors;
}

//------------------------------------------------------------------------------
void BuzzWireTask::SetCurrentToolPose(const KDL::Frame &tool_pose) {

    tool_current_pose_kdl = tool_pose;
    VTKConversions::KDLFrameToVTKMatrix(tool_pose, tool_current_pose);

    // find the center of the ring
    ring_center = tool_current_pose_kdl * KDL::Vector(0.0, 0.0, ring_radius_);


}

//------------------------------------------------------------------------------
void BuzzWireTask::UpdateActors() {

    // -------------------------------------------------------------------------
    // Find closest points and update frames
    tool_current_frame_axes->SetUserMatrix(tool_current_pose);

    ring_actor->SetUserMatrix(tool_current_pose);
    ring_guides_mesh_actor->SetUserMatrix(tool_current_pose);

    FindClosestPoints();

    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose =
            vtkSmartPointer<vtkMatrix4x4>::New();
    VTKConversions::KDLFrameToVTKMatrix(tool_desired_pose_kdl,
                                        tool_desired_pose);
    tool_desired_frame_axes->SetUserMatrix(tool_desired_pose);

    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    double positioning_tolerance = 0.005;
    KDL::Vector destination_cone_position;

    // if the tool is placed close to the starting point while being idle
    //  we can start the task. but we first enable the active constraint.
    if (task_state == TaskState::Idle &&
        (ring_center - start_point).Norm() <
        positioning_tolerance) {
        // Make sure the active constraint is inactive
        if (ac_parameters.active == 0) {
            ac_parameters.active = 1;
            ac_params_changed = true;
        }
    }

    // when the active constraints is suddenly enabled the tool moves
    // significantly. to prevent having a peak in the error always at the
    // beginning of the task data we start the task when the tool is
    // positioned well
    if (task_state == TaskState::Idle &&
        position_error_norm < 0.002 && ac_parameters.active == 1)
    {
        task_state = TaskState::ToEndPoint;
        //increment the repetition number
        number_of_repetition++;
        // save starting time
        start_time = ros::Time::now();
    }

        // If the tool reaches the end point the user needs to go back to
        // the starting point. This counts as a separate repetition of the task
    else if (task_state == TaskState::ToEndPoint &&
             (ring_center - end_point).Norm() <
             positioning_tolerance) {
        task_state = TaskState::ToStartPoint;
        //increment the repetition number
        number_of_repetition++;
        // save starting time
        start_time = ros::Time::now();
    }
        // If the tool reaches the start point while in ToStartPoint state,
        // we can mark the task complete
    else if (task_state == TaskState::ToStartPoint &&
             (ring_center - start_point).Norm() <
             positioning_tolerance) {
        task_state = TaskState::RepetitionComplete;
        ac_parameters.active = 0;
        ac_params_changed = true;
    }

        // User needs to get away from the starting point to switch to idle
        // and in case another repetition is to be performed, the user can
        // flag that by going to the starting position again
    else if (task_state == TaskState::RepetitionComplete &&
             (ring_center - idle_point).Norm() <
             positioning_tolerance) {
        task_state = TaskState::Idle;
    }

    // update the position of the cone according to the state we're in.
    if (task_state == TaskState::Idle) {
        destination_cone_position = start_point;
        destination_cone_actor->GetProperty()->SetColor(Colors::Green);

    } else if (task_state == TaskState::ToStartPoint) {
        destination_cone_position = start_point;
        destination_cone_actor->GetProperty()->SetColor(Colors::DeepPink);

    } else if (task_state == TaskState::ToEndPoint) {
        destination_cone_position = end_point;
        destination_cone_actor->GetProperty()->SetColor(Colors::DeepPink);
    } else if (task_state == TaskState::RepetitionComplete) {
        destination_cone_position = idle_point;
        destination_cone_actor->GetProperty()->SetColor(Colors::Green);

    }

    // show the destination to the user
    double dz = 0.005 +
                0.003 * sin(2 * M_PI * double(destination_cone_counter) / 90);

    destination_cone_counter++;
    destination_cone_actor->SetPosition(destination_cone_position[0],
                                        destination_cone_position[1],
                                        destination_cone_position[2] + dz);

    // -------------------------------------------------------------------------
    // Performance Metrics
    UpdatePositionErrorActor();

    // Populate the task state message
    task_state_msg.task_name = "BuzzWire";
    task_state_msg.task_state = task_state;
    task_state_msg.number_of_repetition = number_of_repetition;
    if (task_state == TaskState::ToStartPoint
        || task_state == TaskState::ToEndPoint) {

        task_state_msg.time_stamp = (ros::Time::now() - start_time).toSec();
        task_state_msg.position_error_norm = position_error_norm;
    } else {
        task_state_msg.time_stamp = 0.0;
        task_state_msg.position_error_norm = 0.0;

    }
}



//------------------------------------------------------------------------------
void BuzzWireTask::FindClosestPoints() {

    //Find the closest cell to the grip point
    double grip_point[3] = {tool_current_pose_kdl.p[0],
                            tool_current_pose_kdl.p[1],
                            tool_current_pose_kdl.p[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};
    double closestPointDist2; //the squared distance to the closest point
    vtkIdType cell_id; //the cell id of the cell containing the closest point
    int subId;

    cellLocator->Update();
    cellLocator->FindClosestPoint(grip_point, closest_point, cell_id, subId,
                                  closestPointDist2);
    closest_point_to_grip_point = KDL::Vector(closest_point[0],
                                              closest_point[1],
                                              closest_point[2]);


    //Find the closest cell to the the central point
    double ring_central_point[3] = {ring_center[0],
                                    ring_center[1],
                                    ring_center[2]};
    cellLocator->Update();
    cellLocator->FindClosestPoint(ring_central_point, closest_point, cell_id,
                                  subId, closestPointDist2);
    closest_point_to_ring_center = KDL::Vector(closest_point[0],
                                               closest_point[1],
                                               closest_point[2]);

    //Find the closest cell to the radial tool point
    KDL::Vector radial_tool_point_kdl = tool_current_pose_kdl *
                                        KDL::Vector(ring_radius_, 0.0,
                                                    ring_radius_);
    double radial_tool_point[3] = {radial_tool_point_kdl[0],
                                   radial_tool_point_kdl[1],
                                   radial_tool_point_kdl[2]};
    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_tool_point, closest_point, cell_id,
                                  subId, closestPointDist2);
    closest_point_to_radial_point = KDL::Vector(closest_point[0],
                                                closest_point[1],
                                                closest_point[2]);
//    // debug using a line
//    line1_source->SetPoint1(ring_central_point);
//    line1_source->SetPoint2(tool_desired_pose_kdl.p[0],
//                            tool_desired_pose_kdl.p[1],
//                            tool_desired_pose_kdl.p[2]);
//    line1_source->Update();
}


//------------------------------------------------------------------------------
KDL::Frame BuzzWireTask::GetDesiredToolPose() {

    CalculatedDesiredToolPose();

    return tool_desired_pose_kdl;
}


//------------------------------------------------------------------------------
void BuzzWireTask::CalculatedDesiredToolPose() {
    // NOTE: All the closest points are on the wire mesh

    // locate the center of the ring
    KDL::Vector ring_center =
            tool_current_pose_kdl * KDL::Vector(0.0, 0.0, ring_radius_);
    // Find the vector from ring center to the corresponding closest point on
    // the wire
    KDL::Vector ring_center_to_cp = closest_point_to_ring_center - ring_center;

    // desired pose only when the ring is close to the wire.if it is too
    // far we don't want fixtures
    if (ring_center_to_cp.Norm() < 3 * ring_radius_) {

//        // activate the guidance if it is not already active
//        if (ac_parameters.active == 0) {
//            ac_params_changed = true;
//            ac_parameters.active = 1;
//        }

        // Desired position is one that puts the center of the wire on the
        // center of the ring.

        //---------------------------------------------------------------------
        // Find the desired position
        KDL::Vector wire_center = ring_center + ring_center_to_cp;
        // Turns out trying to estimate the center of the wire makes things
        // worse so I removed the following term that was used in finding
        // wire_center:
        // wire_radius_ * (ring_center_to_cp/ring_center_to_cp.Norm());

        position_error_norm = (wire_center - ring_center).Norm();
        tool_desired_pose_kdl.p =
                tool_current_pose_kdl.p + wire_center - ring_center;

        //---------------------------------------------------------------------
        // Find the desired orientation
        // We use two vectors to estimate the tangent of the direction of the
        // wire. One is from the grip point (current tool tip) to its closest
        // point on the wire and the other is from a point on the side of the
        // ring (90 deg from the grip point). The estimated tangent is the
        // cross product of these two (after normalization). This is just a
        // quick the non-ideal approximation.
        // Note that we could have used the central point instead of the tool
        // point but that vector gets pretty small and unstable when we're
        // close to the desired pose.
        KDL::Vector radial_point_kdl =
                tool_current_pose_kdl * KDL::Vector(ring_radius_, 0.0,
                                                    ring_radius_);
        KDL::Vector radial_to_cp =
                closest_point_to_radial_point - radial_point_kdl;

        KDL::Vector desired_z, desired_y, desired_x;

        KDL::Vector grip_to_cp =
                closest_point_to_grip_point - tool_current_pose_kdl.p;

        desired_z = grip_to_cp / grip_to_cp.Norm();
        desired_x = -radial_to_cp / radial_to_cp.Norm();
        desired_y = desired_z * desired_x;

        // make sure axes are perpendicular and normal
        desired_y = desired_y / desired_y.Norm();
        desired_x = desired_y * desired_z;
        desired_x = desired_x / desired_x.Norm();
        desired_z = desired_x * desired_y;
        desired_z = desired_z / desired_z.Norm();

        tool_desired_pose_kdl.M = KDL::Rotation(desired_x, desired_y,
                                                desired_z);
        //        tool_desired_pose_kdl.M = tool_current_pose_kdl.M;
    } else {
//        // deactivate the guidance if not already inactive
//        if (ac_parameters.active == 1) {
//            ac_params_changed = true;
//            ac_parameters.active = 0;
//        }

        tool_desired_pose_kdl = tool_current_pose_kdl;
        // due to the delay in teleop loop this will some create forces if the
        // guidance is still active
    }

}


//------------------------------------------------------------------------------
bool BuzzWireTask::IsACParamChanged() {
    return ac_params_changed;
}


//------------------------------------------------------------------------------
active_constraints::ActiveConstraintParameters BuzzWireTask::GetACParameters() {

    ac_params_changed = false;
    // assuming once we read it we can consider it unchanged
    return ac_parameters;
}


//------------------------------------------------------------------------------
void BuzzWireTask::UpdatePositionErrorActor() {

    double max_error = 0.004;
    double error_ratio = position_error_norm / max_error;
    if (error_ratio > 1.0)
        error_ratio = 1.0;

    error_sphere_actor->GetProperty()->SetColor(error_ratio, 1 - error_ratio,
                                                0.1);

}

teleop_vision::TaskState BuzzWireTask::GetTaskStateMsg() {
    return task_state_msg;
}

void BuzzWireTask::Reset() {
    number_of_repetition = 0;
    task_state = TaskState::RepetitionComplete;
}

void BuzzWireTask::RepeatLastAcquisition() {
    number_of_repetition--;
    task_state = TaskState::RepetitionComplete;
}

