//
// Created by nima on 4/18/17.
//

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include "KidneyTask.h"


namespace COLORS {
    double Red[3] {1.0, 0.1, 0.03};
    double OrangeRed[3] {1.0, 0.27, 0.03};
    double Gold[3] {1.0, 0.84, 0.0};
    double Green[3] {0.0, 0.9, 0.03};
    double Pink[3] {1.0, 0.0, 1.0};
    double Orange[3] {0.9, 0.4, 0.1};
    double Gray [3] {0.4, 0.4, 0.4};
    double Turquoise[3]	{0.25, 0.88, 0.82};
    double DeepPink[3] {1.0, 0.08, 0.58};
};

KidneyTask::KidneyTask(const std::string stl_file_dir,
                       const bool show_ref_frames, const bool biman,
                       const bool with_guidance)
        :
        VTKTask(show_ref_frames, biman, with_guidance),
        stl_files_dir(stl_file_dir),
//        show_ref_frames(show_ref_frames),
//        bimanual(biman),
//        with_guidance(with_guidance),
        destination_ring_counter(0),
        ac_params_changed(true),
        task_state(KidneyTaskState::Idle),
        number_of_repetition(0),
        n_score_history(10),
        idle_point(KDL::Vector(-0.006, 0.026, 0.029)),
        start_point(KDL::Vector(0.0019, 0.031, 0.030)),
        end_point(KDL::Vector(0.034, 0.043, 0.053)) {

    dInitODE ();

    InitODE();


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

    ring_actor[0] = vtkSmartPointer<vtkActor>::New();
    tool_current_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_current_pose[0] = vtkSmartPointer<vtkMatrix4x4>::New();

    if(bimanual){
        ring_actor[1] = vtkSmartPointer<vtkActor>::New();
        tool_current_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
        tool_desired_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
        tool_current_pose[1] = vtkSmartPointer<vtkMatrix4x4>::New();

    }

    destination_ring_actor= vtkSmartPointer<vtkActor>::New();

    cellLocator = vtkSmartPointer<vtkCellLocator>::New();

    line1_source = vtkSmartPointer<vtkLineSource>::New();

    line2_source = vtkSmartPointer<vtkLineSource>::New();

    line1_actor = vtkSmartPointer<vtkActor>::New();

    line2_actor = vtkSmartPointer<vtkActor>::New();

    d_cube_actor = vtkSmartPointer<vtkActor>::New();

    // -------------------------------------------------------------------------
    // TOOL RINGS
    ring_radius = 0.004;
    double ring_cross_section_radius = 0.0005;
    double source_scales = 0.006;

    vtkSmartPointer<vtkParametricTorus> parametricObject =
            vtkSmartPointer<vtkParametricTorus>::New();
    parametricObject->SetCrossSectionRadius(
            ring_cross_section_radius / source_scales);
    parametricObject->SetRingRadius(ring_radius / source_scales);

    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
            vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();

    // to transform the data
    vtkSmartPointer<vtkTransformPolyDataFilter>
            ring_local_transform_filter[2];
    vtkSmartPointer<vtkTransform> ring_local_transform[2];
    vtkSmartPointer<vtkPolyDataMapper> ring_mapper[2];


    for (int k = 0; k < 1 + (int)bimanual; ++k) {

        ring_local_transform_filter[k] =
                vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        ring_local_transform_filter[k]->SetInputConnection(
                parametricFunctionSource->GetOutputPort());

        ring_local_transform[k] = vtkSmartPointer<vtkTransform>::New();

        if(k == 0){
            ring_local_transform[k]->RotateX(90);
            ring_local_transform[k]->Translate(0.0, ring_radius /
                                                    source_scales, 0.0);
        }
        else{
//            ring_local_transform[k]->RotateX(0);
            ring_local_transform[k]->Translate(0.0, ring_radius /
                                                    source_scales, 0.0);
        }

        ring_local_transform_filter[k]->SetTransform(ring_local_transform[k]);
        ring_local_transform_filter[k]->Update();

        ring_mapper[k] = vtkSmartPointer<vtkPolyDataMapper>::New();
        ring_mapper[k]->SetInputConnection(
                ring_local_transform_filter[k]->GetOutputPort());

        ring_actor[k]->SetMapper(ring_mapper[k]);
        ring_actor[k]->SetScale(source_scales);
        ring_actor[k]->GetProperty()->SetColor(COLORS::Turquoise);
        ring_actor[k]->GetProperty()->SetSpecular(0.7);
    }

    // -------------------------------------------------------------------------
    // Destination ring

    vtkSmartPointer<vtkPolyDataMapper> destination_ring_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    destination_ring_mapper->SetInputConnection(
            parametricFunctionSource->GetOutputPort());
    destination_ring_actor->SetMapper(destination_ring_mapper);
    destination_ring_actor->SetScale(0.004);
    destination_ring_actor->RotateX(90);
    destination_ring_actor->RotateY(-60);
    destination_ring_actor->GetProperty()->SetColor(COLORS::Green);
    destination_ring_actor->GetProperty()->SetOpacity(0.5);

    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    for (int k = 0; k < 1 + (int)bimanual; ++k) {

        tool_current_frame_axes[k]->SetXAxisLabelText("");
        tool_current_frame_axes[k]->SetYAxisLabelText("");
        tool_current_frame_axes[k]->SetZAxisLabelText("");
        tool_current_frame_axes[k]->SetTotalLength(0.007, 0.007, 0.007);
        tool_current_frame_axes[k]->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

        tool_desired_frame_axes[k]->SetXAxisLabelText("");
        tool_desired_frame_axes[k]->SetYAxisLabelText("");
        tool_desired_frame_axes[k]->SetZAxisLabelText("");
        tool_desired_frame_axes[k]->SetTotalLength(0.007, 0.007, 0.007);
        tool_desired_frame_axes[k]->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);

    }
    // -------------------------------------------------------------------------
    // Stand MESH hq
    std::stringstream input_file_dir;
    input_file_dir << stl_files_dir << std::string("tumor2.stl");

    vtkSmartPointer<vtkSTLReader> stand_mesh_reader =
            vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << input_file_dir.str() << std::endl;
    stand_mesh_reader->SetFileName(input_file_dir.str().c_str());
    stand_mesh_reader->Update();

    // transform
    vtkSmartPointer<vtkTransform> kidney_mesh_transform =
            vtkSmartPointer<vtkTransform>::New();
    kidney_mesh_transform->Translate(0.050, 0.020, 0.0);
//    kidney_mesh_transform->RotateX(180);
//    kidney_mesh_transform->RotateZ(150);
    kidney_mesh_transform->Scale(0.01, 0.01, 0.01);

    vtkSmartPointer<vtkTransformPolyDataFilter> kidney_mesh_transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    kidney_mesh_transformFilter->SetInputConnection(
            stand_mesh_reader->GetOutputPort());
    kidney_mesh_transformFilter->SetTransform(kidney_mesh_transform);
    kidney_mesh_transformFilter->Update();


    vtkSmartPointer<vtkPolyDataMapper> kidney_mesh_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    kidney_mesh_mapper->SetInputConnection(
            kidney_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> kidney_mesh_actor =
            vtkSmartPointer<vtkActor>::New();
    kidney_mesh_actor->SetMapper(kidney_mesh_mapper);
    kidney_mesh_actor->GetProperty()->SetColor(COLORS::Red);
    kidney_mesh_actor->GetProperty()->SetOpacity(1.0);
    //    stand_mesh_actor->GetProperty()->SetSpecular(0.8);


    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(kidney_mesh_transformFilter->GetOutput());
    cellLocator->BuildLocator();

    // -------------------------------------------------------------------------
    // Create a cube for the floor
    vtkSmartPointer<vtkCubeSource> floor_source =
            vtkSmartPointer<vtkCubeSource>::New();
    double floor_dimensions[3] = {0.1, 0.09, 0.001};
    floor_source->SetXLength(floor_dimensions[0]);
    floor_source->SetYLength(floor_dimensions[1]);
    floor_source->SetZLength(floor_dimensions[2]);
    vtkSmartPointer<vtkPolyDataMapper> floor_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    floor_mapper->SetInputConnection(floor_source->GetOutputPort());
    vtkSmartPointer<vtkActor> floor_actor = vtkSmartPointer<vtkActor>::New();
    floor_actor->SetMapper(floor_mapper);
    floor_actor->SetPosition(floor_dimensions[0] / 2, floor_dimensions[1] / 2,
                             -floor_dimensions[2]);
    floor_actor->GetProperty()->SetOpacity(0.3);
    floor_actor->GetProperty()->SetColor(COLORS::Pink);

    // -------------------------------------------------------------------------
    // Lines
    vtkSmartPointer<vtkPolyDataMapper> line1_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    line1_actor->SetMapper(line1_mapper);
    line1_actor->GetProperty()->SetLineWidth(3);
//    line1_actor->GetProperty()->SetColor(COLORS::DeepPink);
    line1_actor->GetProperty()->SetOpacity(0.8);

    vtkSmartPointer<vtkPolyDataMapper> line2_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    line2_mapper->SetInputConnection(line2_source->GetOutputPort());

    line2_actor->SetMapper(line2_mapper);
    line2_actor->GetProperty()->SetLineWidth(3);
//    line2_actor->GetProperty()->SetColor(COLORS::DeepPink);
    line2_actor->GetProperty()->SetOpacity(0.8);

//    // -------------------------------------------------------------------------
//    // destination cone
//    vtkSmartPointer<vtkConeSource> destination_cone_source =
//            vtkSmartPointer<vtkConeSource>::New();
//    destination_cone_source->SetRadius(0.002 / source_scales);
//    destination_cone_source->SetHeight(0.006 / source_scales);
//    destination_cone_source->SetResolution(12);
//
//    vtkSmartPointer<vtkPolyDataMapper> destination_cone_mapper =
//            vtkSmartPointer<vtkPolyDataMapper>::New();
//    destination_cone_mapper->SetInputConnection(
//            destination_cone_source->GetOutputPort());
//    destination_cone_actor = vtkSmartPointer<vtkActor>::New();
//    destination_cone_actor->SetMapper(destination_cone_mapper);
//    destination_cone_actor->SetScale(source_scales);
//    destination_cone_actor->GetProperty()->SetColor(COLORS::Green);
//    destination_cone_actor->GetProperty()->SetOpacity(0.5);
//    destination_cone_actor->RotateY(90);
//    destination_cone_actor->RotateZ(30);
//
//    destination_cone_actor->SetPosition(idle_point[0], idle_point[1],
//                                        idle_point[2]);

    // -------------------------------------------------------------------------
    // TEXTS
    cornerAnnotation =
            vtkSmartPointer<vtkCornerAnnotation>::New();
    cornerAnnotation->SetLinearFontScaleFactor( 2 );
    cornerAnnotation->SetNonlinearFontScaleFactor( 1 );
    cornerAnnotation->SetMaximumFontSize( 30 );
    //        cornerAnnotation->SetText( 0, "lower left" );
    cornerAnnotation->SetText( 1, "Scores: " );
    //        cornerAnnotation->SetText( 2, "upper left" );
    //    cornerAnnotation->GetTextProperty()->SetColor( 1, 0, 0 );



    // -------------------------------------------------------------------------
    // Error history spheres

    vtkSmartPointer<vtkSphereSource>  source =
            vtkSmartPointer<vtkSphereSource>::New();

    source->SetRadius(0.002);
    source->SetPhiResolution(15);
    source->SetThetaResolution(15);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(source->GetOutputPort());

    for (int i = 0; i < n_score_history; ++i) {

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        actor->GetProperty()->SetColor(COLORS::Gray);
        actor->SetPosition(0.02 - (double)i * 0.006, 0.1 - (double)i * 0.0003,
                           0.01);
        score_sphere_actors.push_back(actor);
    }

    // -------------------------------------------------------------------------
//    vtkSmartPointer<vtkCubeSource> cube_source =
//            vtkSmartPointer<vtkCubeSource>::New();
    vtkSmartPointer<vtkSphereSource>  basll_source =
            vtkSmartPointer<vtkSphereSource>::New();

//    basll_source->SetRadius(0.01);
    basll_source->SetPhiResolution(15);
    basll_source->SetThetaResolution(15);
    vtkSmartPointer<vtkPolyDataMapper> cube_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    cube_mapper->SetInputConnection(basll_source->GetOutputPort());
    d_cube_actor->SetMapper(cube_mapper);
    d_cube_actor->SetScale(2*source_scales);

    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);

        for (int k = 0; k < 1 + (int)bimanual; ++k) {
            actors.push_back(tool_current_frame_axes[k]);
            actors.push_back(tool_desired_frame_axes[k]);
        }
    }


    actors.push_back(kidney_mesh_actor);
//    actors.push_back(floor_actor);
    actors.push_back(ring_actor[0]);
    if(bimanual){
        actors.push_back(ring_actor[1]);
        actors.push_back(line1_actor);
        actors.push_back(line2_actor);
    }
//    actors.push_back(destination_cone_actor);
    actors.push_back(destination_ring_actor);
    for (int j = 0; j < score_sphere_actors.size(); ++j) {
        actors.push_back(score_sphere_actors[j]);
    }
    actors.push_back(d_cube_actor);

    //    actors.push_back(ring_guides_mesh_actor);
    //    actors.push_back(cornerAnnotation);


}

//------------------------------------------------------------------------------
//std::vector<vtkSmartPointer<vtkProp> > KidneyTask::GetActors() {
//    return actors;
//}

//------------------------------------------------------------------------------
void KidneyTask::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                           const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

//------------------------------------------------------------------------------
void KidneyTask::UpdateActors() {

    SimLoopODE();

    // -------------------------------------------------------------------------
    // Find closest points and update frames
    for (int k = 0; k < 1 + (int)bimanual; ++k) {
        tool_current_frame_axes[k]->SetUserMatrix(tool_current_pose[k]);
        ring_actor[k]->SetUserMatrix(tool_current_pose[k]);
    }

//    ring_guides_mesh_actor->SetUserMatrix(tool_current_pose[0]);


    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose[2];

    for (int k = 0; k < 1 + (int)bimanual; ++k) {
        tool_desired_pose[k] =
                vtkSmartPointer<vtkMatrix4x4>::New();
        VTKConversions::KDLFrameToVTKMatrix(tool_desired_pose_kdl[k],
                                            tool_desired_pose[k]);
        tool_desired_frame_axes[k]->SetUserMatrix(tool_desired_pose[k]);
    }

    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    double positioning_tolerance = 0.003;
    KDL::Vector destination_cone_position;


    // when the active constraints is suddenly enabled the tool makes some
    // initial large movements. To prevent always having a peak at the
    // beginning of the error, data we first activate the active constraint
    // and wait till the error is small before we start the task.
    //
    // First: if the tool is placed close to the starting point while being idle
    // we enable the active constraint.
    if (task_state == KidneyTaskState::Idle && with_guidance &&
        (ring_center[0] - start_point).Norm() <
        positioning_tolerance) {
        // Make sure the active constraint is inactive
        if (ac_parameters.active == 0) {
            ac_parameters.active = 1;
            ac_params_changed = true;
        }
    }

    // then when the error is small we start the task.
    // if guidance is off, then we don't need a restriction on the error
    // sorry it is totally unreadable!
    if (task_state == KidneyTaskState::Idle
        && ( (position_error_norm[0] < 0.001 && ac_parameters.active == 1)
             ||
             (!with_guidance && (ring_center[0] - start_point).Norm() <
                                positioning_tolerance)  ))
    {
        task_state = KidneyTaskState::ToEndPoint;
        //increment the repetition number
        number_of_repetition ++;
        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
    }

        // If the tool reaches the end point the user needs to go back to
        // the starting point. This counts as a separate repetition of the task
    else if (task_state == KidneyTaskState::ToEndPoint &&
             (ring_center[0] - end_point).Norm() <
             positioning_tolerance) {
        task_state = KidneyTaskState::ToStartPoint;
        //increment the repetition number
        number_of_repetition++;

        // calculate and save the score of this repetition
        CalculateAndSaveError();

        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
    }
        // If the tool reaches the start point while in ToStartPoint state,
        // we can mark the task complete
    else if (task_state == KidneyTaskState::ToStartPoint &&
             (ring_center[0] - start_point).Norm() <
             positioning_tolerance) {
        task_state = KidneyTaskState::RepetitionComplete;
        // calculate and save the score of this repetition
        CalculateAndSaveError();

        ac_parameters.active = 0;
        ac_params_changed = true;
    }

        // User needs to get away from the starting point to switch to idle
        // and in case another repetition is to be performed, the user can
        // flag that by going to the starting position again
    else if (task_state == KidneyTaskState::RepetitionComplete &&
             (ring_center[0] - idle_point).Norm() <
             positioning_tolerance) {
        task_state = KidneyTaskState::Idle;
    }

    // update the position of the cone according to the state we're in.
    if (task_state == KidneyTaskState::Idle) {
        destination_cone_position = start_point;
        destination_ring_actor->GetProperty()->SetColor(COLORS::Green);

    } else if (task_state == KidneyTaskState::ToStartPoint) {
        destination_cone_position = start_point;
        destination_ring_actor->GetProperty()->SetColor(COLORS::DeepPink);

    } else if (task_state == KidneyTaskState::ToEndPoint) {
        destination_cone_position = end_point;
        destination_ring_actor->GetProperty()->SetColor(COLORS::DeepPink);
    } else if (task_state == KidneyTaskState::RepetitionComplete) {
        destination_cone_position = idle_point;
        destination_ring_actor->GetProperty()->SetColor(COLORS::Green);

    }

    // show the destination to the user
    double dt = sin(2 * M_PI * double(destination_ring_counter) / 70);
    destination_ring_counter++;
    destination_ring_actor->SetScale(0.006 + 0.001*dt);

    destination_ring_actor->SetPosition(destination_cone_position[0],
                                        destination_cone_position[1],
                                        destination_cone_position[2]);
    // -------------------------------------------------------------------------
    // Performance Metrics
    UpdateTubeColor();

    // Populate the task state message
    task_state_msg.task_name = "BuzzWire";
    task_state_msg.task_state = (uint8_t)task_state;
    task_state_msg.number_of_repetition = number_of_repetition;
    if (task_state == KidneyTaskState::ToStartPoint
        || task_state == KidneyTaskState::ToEndPoint) {

        task_state_msg.time_stamp = (ros::Time::now() - start_time).toSec();
        task_state_msg.error_field_1 = position_error_norm[0];
        task_state_msg.error_field_1 = orientation_error_norm[0];
        //        if(bimanual)
        //            task_state_msg.error_field_2 = position_error_norm[1];

        // calculate score to show to user
        if (bimanual) {
            posit_error_sum +=
                    0.5 * (position_error_norm[0] +
                           position_error_norm[1]);
            orient_error_sum +=
                    0.5 *(orientation_error_norm[0] +
                          orientation_error_norm[1]);
        }
        else {
            posit_error_sum += position_error_norm[0];
            orient_error_sum += orientation_error_norm[0];
        }

        if(posit_error_max < position_error_norm[0])
            posit_error_max = position_error_norm[0];

        sample_count++;

    } else {
        task_state_msg.time_stamp = 0.0;
        task_state_msg.error_field_1 = 0.0;
        task_state_msg.error_field_2 = 0.0;
    }


    if(bimanual) {
        // change connection lines colors according to ring1 to ring2's distance
        double rings_distance = (ring_center[1] - ring_center[0]).Norm();
        double ideal_distance = 0.007;
        double error_ratio = 3 * fabs(rings_distance - ideal_distance)
                             / ideal_distance;
        if (error_ratio > 1.0)
            error_ratio = 1.0;
        line1_actor->GetProperty()->SetColor(0.9, 0.9 - 0.7 * error_ratio,
                                             0.9 - 0.7 * error_ratio);
        line2_actor->GetProperty()->SetColor(0.9, 0.9 - 0.7 * error_ratio,
                                             0.9 - 0.7 * error_ratio);
        line1_source->Update();
        line2_source->Update();
    }
}



//------------------------------------------------------------------------------
void KidneyTask::CalculatedDesiredToolPose() {
    // NOTE: All the closest points are on the wire mesh

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
    // WHen I added the second tool things got a little bit messy!
    // Since I had to rotate the sing for 90 degrees the diserd axes
    // were different for each ring and it ended up with too many
    // hardcoded transforms, I will hopefully clean this out later. TODO

    for (int k = 0; k < 1 + (int)bimanual; ++k) {

        // make a copy of the current pose
        KDL::Frame tool_current_pose= *tool_current_pose_kdl[k];

        //Find the closest cell to the grip point
        double grip_point[3] = {(tool_current_pose).p[0],
                                (tool_current_pose).p[1],
                                (tool_current_pose).p[2]};

        double closest_point[3] = {0.0, 0.0, 0.0};
        double closestPointDist2; //the squared distance to the closest point
        vtkIdType cell_id; //the cell id of the cell containing the closest point
        int subId;

        cellLocator->Update();
        cellLocator->FindClosestPoint(grip_point, closest_point, cell_id, subId,
                                      closestPointDist2);
        closest_point_to_grip_point[k] = KDL::Vector(closest_point[0],
                                                     closest_point[1],
                                                     closest_point[2]);


        //Find the closest cell to the the central point
        double ring_central_point[3] = {ring_center[k][0],
                                        ring_center[k][1],
                                        ring_center[k][2]};
        cellLocator->Update();
        cellLocator->FindClosestPoint(ring_central_point, closest_point,
                                      cell_id,
                                      subId, closestPointDist2);
        closest_point_to_ring_center[k] = KDL::Vector(closest_point[0],
                                                      closest_point[1],
                                                      closest_point[2]);

        //Find the closest cell to the radial tool point
        KDL::Vector radial_tool_point_kdl;
        if(k==0)
            radial_tool_point_kdl =
                    tool_current_pose *
                    KDL::Vector(ring_radius, 0.0, ring_radius);
        else
            radial_tool_point_kdl =
                    tool_current_pose *
                    KDL::Vector(ring_radius, ring_radius, 0.0);

        double radial_tool_point[3] = {radial_tool_point_kdl[0],
                                       radial_tool_point_kdl[1],
                                       radial_tool_point_kdl[2]};

        cellLocator->Update();
        cellLocator->FindClosestPoint(radial_tool_point, closest_point, cell_id,
                                      subId, closestPointDist2);
        closest_point_to_radial_point[k] = KDL::Vector(closest_point[0],
                                                       closest_point[1],
                                                       closest_point[2]);


        // Find the vector from ring center to the corresponding closest point on
        // the wire
        KDL::Vector ring_center_to_cp =
                closest_point_to_ring_center[k] - ring_center[k];

        // desired pose only when the ring is close to the wire.if it is too
        // far we don't want fixtures
        if (ring_center_to_cp.Norm() < 5 * ring_radius) {


            // Desired position is one that puts the center of the wire on the
            // center of the ring.

            //---------------------------------------------------------------------
            // Find the desired position
            KDL::Vector wire_center = ring_center[k] + ring_center_to_cp;
            // Turns out trying to estimate the center of the wire makes things
            // worse so I removed the following term that was used in finding
            // wire_center:
            // wire_radius_ * (ring_center_to_cp/ring_center_to_cp.Norm());

            tool_desired_pose_kdl[k].p =
                    (tool_current_pose).p + wire_center -
                    ring_center[k];



            KDL::Vector radial_to_cp =
                    closest_point_to_radial_point[k] - radial_tool_point_kdl;

            KDL::Vector desired_z, desired_y, desired_x;

            KDL::Vector grip_to_cp =
                    closest_point_to_grip_point[k] -
                    (tool_current_pose).p;
            if(k==0) {
                desired_z = grip_to_cp / grip_to_cp.Norm();
                desired_x = -radial_to_cp / radial_to_cp.Norm();
                desired_y = desired_z * desired_x;

                // make sure axes are perpendicular and normal
                desired_y = desired_y / desired_y.Norm();
                desired_x = desired_y * desired_z;
                desired_x = desired_x / desired_x.Norm();
                desired_z = desired_x * desired_y;
                desired_z = desired_z / desired_z.Norm();
            }
            else {
                desired_y = grip_to_cp / grip_to_cp.Norm();
                desired_x = -radial_to_cp / radial_to_cp.Norm();
                desired_z = desired_x * desired_y;

                // make sure axes are perpendicular and normal
                desired_z = desired_z / desired_z.Norm();
                desired_x = desired_y * desired_z;
                desired_x = desired_x / desired_x.Norm();
                desired_y = desired_z * desired_x;
                desired_y = desired_y / desired_y.Norm();
            }
            tool_desired_pose_kdl[k].M = KDL::Rotation(desired_x, desired_y,
                                                       desired_z);

            //------------------------------------------------------------------
            // Calculate errors
            position_error_norm[k] = (wire_center - ring_center[k]).Norm();
            KDL::Vector rpy;
            (tool_desired_pose_kdl[k].M *
             (tool_current_pose).M.Inverse() ).GetRPY(rpy[0],
                                                      rpy[1],
                                                      rpy[2]);
            orientation_error_norm[k] = rpy.Norm();


        } else {
            tool_desired_pose_kdl[k] = tool_current_pose;
            // due to the delay in teleop loop this will create some wrneches if
            // the guidance is still active
        }


        // draw the connection lines in bimanual case
        if(bimanual) {
            KDL::Vector distal_tool_point_kdl;
            if (k == 0)
                distal_tool_point_kdl =
                        tool_current_pose *
                        KDL::Vector(0.0, 0.0, 2 * ring_radius);
            else
                distal_tool_point_kdl =
                        tool_current_pose *
                        KDL::Vector(0.0, 2 * ring_radius, 0.0);

            if (k == 0) {
                line1_source->SetPoint1(grip_point);
                line2_source->SetPoint1(distal_tool_point_kdl[0],
                                        distal_tool_point_kdl[1],
                                        distal_tool_point_kdl[2]);

            } else {
                line1_source->SetPoint2(grip_point);
                line2_source->SetPoint2(distal_tool_point_kdl[0],
                                        distal_tool_point_kdl[1],
                                        distal_tool_point_kdl[2]);
            }
        }

    }

}


//------------------------------------------------------------------------------
bool KidneyTask::IsACParamChanged() {
    return ac_params_changed;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters KidneyTask::GetACParameters() {

    ac_params_changed = false;
    // assuming once we read it we can consider it unchanged
    return ac_parameters;
}


//------------------------------------------------------------------------------
void KidneyTask::UpdateTubeColor() {

    double max_pos_error = 0.002;
    double max_orient_error = 0.3;
    // orientation error is tricky to perceive, so we weigh it half the
    // position error
    double error_ratio = ( (orientation_error_norm[0] / max_orient_error)
                           + 2* (position_error_norm[0] / max_pos_error)) /3;

    if (error_ratio > 1.3)
        error_ratio = 1.3;
    else if(error_ratio < 0.3)
        error_ratio = 0.3;

//    score_sphere_actors->GetProperty()->SetColor(error_ratio, 1 - error_ratio,
//                                                0.1);
//    if(task_state== KidneyTaskState::ToEndPoint
//       || task_state== KidneyTaskState::ToStartPoint)
//        tube_mesh_actor->GetProperty()->SetColor(0.9,
//                                                 0.5- 0.4*(error_ratio-0.3),
//                                                 0.1);

}

custom_msgs::TaskState KidneyTask::GetTaskStateMsg() {
    return task_state_msg;
}

void KidneyTask::ResetTask() {
    ROS_INFO("Resetting the task.");
    number_of_repetition = 0;
    task_state = KidneyTaskState::RepetitionComplete;
    ResetOnGoingEvaluation();
    ResetScoreHistory();
}

void KidneyTask::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
    if(task_state== KidneyTaskState::ToEndPoint ||
       task_state == KidneyTaskState::ToStartPoint){

        ResetOnGoingEvaluation();
        if(number_of_repetition>0)
            number_of_repetition--;
        task_state = KidneyTaskState::RepetitionComplete;

    }
}


void KidneyTask::FindAndPublishDesiredToolPose() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            ("/PSM1/tool_pose_desired", 10);
    if(bimanual)
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        VTKConversions::KDLFrameToVTKMatrix(*tool_current_pose_kdl[0],
                                            tool_current_pose[0]);
        // find the center of the ring
        ring_center[0] = *tool_current_pose_kdl[0] *
                         KDL::Vector(0.0, 0.0,ring_radius);

        if(bimanual){
            VTKConversions::KDLFrameToVTKMatrix(*tool_current_pose_kdl[1],
                                                tool_current_pose[1]);
            ring_center[1] = *tool_current_pose_kdl[1] *
                             KDL::Vector(0.0, ring_radius, 0.0);
        }


        CalculatedDesiredToolPose();

        // publish desired poses
        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/task_space";
            pose_msg.header.stamp = ros::Time::now();
            // publish
            pub_desired[n_arm].publish(pose_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}

void KidneyTask::CalculateAndSaveError() {

    double duration = (ros::Time::now() - start_time).toSec();
    double posit_error_avg = posit_error_sum/(double)sample_count;
    double orient_error_avg = orient_error_sum/(double)sample_count;

    double posit_error_avg_ideal = 0.0005;
    double orient_error_avg_ideal = 0.35;
    double posit_error_max_ideal = 0.0015;
    double duration_ideal = 9.0;

    // put a threshold on the values
    if (posit_error_avg < posit_error_avg_ideal)
        posit_error_avg = posit_error_avg_ideal;

    if (orient_error_avg < orient_error_avg_ideal)
        orient_error_avg = orient_error_avg_ideal;

    if (posit_error_max < posit_error_max_ideal)
        posit_error_max = posit_error_max_ideal;

    if (duration < duration_ideal)
        duration = duration_ideal;

    double score = ( posit_error_avg_ideal/posit_error_avg
                     + posit_error_max_ideal/posit_error_max
                     + duration_ideal/duration
                     + orient_error_avg_ideal/orient_error_avg)
                   * 100 / 4;

    // when the history gets full we start a new set
    if (score_history.size() == n_score_history)
        ResetScoreHistory();

    score_history.push_back(score);
    score_history_colors.push_back(GetScoreColor(score));

    // update spheres' color
    for (int i = 0; i < score_history.size(); ++i) {
        score_sphere_actors[i]->GetProperty()->SetColor(score_history_colors[i]);

    }

    ROS_INFO("posit_error_max: %f", posit_error_max);
    ROS_INFO("duration: %f", duration);
    ROS_INFO("posit_error_avg: %f", posit_error_avg);
    ROS_INFO("orient_error_avg: %f", orient_error_avg);
    ROS_INFO("Score: %f", score);
    ROS_INFO("  ");
}

double * KidneyTask::GetScoreColor(const double score) {

    //decide the color
    if(score > 90){
        return COLORS::Green;
    }
    else if (score > 80)
        return COLORS::Gold;
    else if(score > 60)
        return COLORS::Orange;
    else
        return COLORS::Red;

}

void KidneyTask::ResetOnGoingEvaluation() {
    posit_error_sum = 0.0;
    posit_error_max = 0.0;
    orient_error_sum = 0.0;
    sample_count = 0;
}

void KidneyTask::ResetScoreHistory() {
    score_history.clear();
    score_history_colors.clear();

    // reset colors to gray
    for (int i = 0; i < n_score_history; ++i) {
        score_sphere_actors[i]->GetProperty()->SetColor(COLORS::Gray);

    }

}

void KidneyTask::InitODE() {


    // Create a new, empty world and assign its ID number to World. Most applications will only need one world.

//    World = dWorldCreate();

    // Create a new collision space and assign its ID number to Space, passing 0 instead of an existing dSpaceID.
    // There are three different types of collision spaces we could create here depending on the number of objects
    // in the world but dSimpleSpaceCreate is fine for a small number of objects. If there were more objects we
    // would be using dHashSpaceCreate or dQuadTreeSpaceCreate (look these up in the ODE docs)

    Space = dSimpleSpaceCreate(0);

    // Create a joint group object and assign its ID number to contactgroup. dJointGroupCreate used to have a
    // max_size parameter but it is no longer used so we just pass 0 as its argument.

//    contactgroup = dJointGroupCreate(0);

    // Create a ground plane in our collision space by passing Space as the first argument to dCreatePlane.
    // The next four parameters are the planes normal (a, b, c) and distance (d) according to the plane
    // equation a*x+b*y+c*z=d and must have length 1

    dCreatePlane(Space, 0, 0, 1, 0);

    // Now we set the gravity vector for our world by passing World as the first argument to dWorldSetGravity.
    // Earth's gravity vector would be (0, -9.81, 0) assuming that +Y is up. I found that a lighter gravity looked
    // more realistic in this case.

    dWorldSetGravity(World, 0.0, 0,  (float)-9.8);

    // These next two functions control how much error correcting and constraint force mixing occurs in the world.
    // Don't worry about these for now as they are set to the default values and we could happily delete them from
    // this example. Different values, however, can drastically change the behaviour of the objects colliding, so
    // I suggest you look up the full info on them in the ODE docs.

//    dWorldSetERP(World, 0.2);
//
    dWorldSetCFM(World, 1e-5);

    // This function sets the velocity that interpenetrating objects will separate at. The default value is infinity.
    dWorldSetContactMaxCorrectingVel(World, 0.9);

    // This function sets the depth of the surface layer around the world objects. Contacts are allowed to sink into
    // each other up to this depth. Setting it to a small value reduces the amount of jittering between contacting
    // objects, the default value is 0.

    dWorldSetContactSurfaceLayer(World, 0.001);

    // To save some CPU time we set the auto disable flag to 1. This means that objects that have come to rest (based
    // on their current linear and angular velocity) will no longer participate in the simulation, unless acted upon
    // by a moving object. If you do not want to use this feature then set the flag to 0. You can also manually enable
    // or disable objects using dBodyEnable and dBodyDisable, see the docs for more info on this.

    dWorldSetAutoDisableFlag(World, 1);

    // This brings us to the end of the world settings, now we have to initialize the objects themselves.
    // Create a new body for our object in the world and get its ID.

    Objct.Body = dBodyCreate(World);



    // Next we set the position of the new body

    dBodySetPosition(Objct.Body, 0.001, 0.02, 0.05);

    // Here I have set the initial linear velocity to stationary and let gravity do the work, but you can experiment
    // with the velocity vector to change the starting behaviour. You can also set the rotational velocity for the new
    // body using dBodySetAngularVel which takes the same parameters.

    double tempVect[3] = {0.0, 0.0, 0.0};

    dBodySetLinearVel(Objct.Body, tempVect[0], tempVect[1], tempVect[2]);

    // To start the object with a different rotation each time the program runs we create a new matrix called R and use
    // the function dRFromAxisAndAngle to create a random initial rotation before passing this matrix to dBodySetRotation.
    dMatrix3 R;

    dRFromAxisAndAngle(R, dRandReal() * 2.0 - 1.0,

                       dRandReal() * 2.0 - 1.0,

                       dRandReal() * 2.0 - 1.0,

                       dRandReal() * 10.0 - 5.0);

    dBodySetRotation(Objct.Body, R);

    // At this point we could add our own user data using dBodySetData but in this example it isn't used.
    size_t i = 0;

    dBodySetData(Objct.Body, (void*)i);

    // Now we need to create a box mass to go with our geom. First we create a new dMass structure (the internals
    // of which aren't important at the moment) then create an array of 3 float (dReal) values and set them
    // to the side lengths of our box along the x, y and z axes. We then pass the both of these to dMassSetBox with a
    // pre-defined DENSITY value of 0.5 in this case.

    dMass m;

    dReal sides[3];

    sides[0] = 0.02;

    sides[1] = 0.02;

    sides[2] = 0.02;
    double DENSITY = 0.005;
//    dMassSetBox(&m, DENSITY, sides[0], sides[1], sides[2]);
    dMassSetSphere(&m, DENSITY, 0.01);
    // We can then apply this mass to our objects body.
    dBodySetMass(Objct.Body, &m);

    // Here we create the actual geom object using dCreateBox. Note that this also adds the geom to our
    // collision space and sets the size of the geom to that of our box mass.
//    Objct.Geom[0] = dCreateBox(Space, sides[0], sides[1], sides[2]);
    Objct.Geom[0] = dCreateSphere(Space, 0.01);

    // And lastly we want to associate the body with the geom using dGeomSetBody. Setting a body on a geom automatically
    // combines the position vector and rotation matrix of the body and geom so that setting the position or orientation
    // of one will set the value for both objects. The ODE docs have a lot more to say about the geom functions.
    dGeomSetBody(Objct.Geom[0], Objct.Body);



}

void KidneyTask::CloseODE() {

    // Destroy all joints in our joint group
    dJointGroupDestroy(contactgroup);

    // Destroy the collision space. When a space is destroyed, and its cleanup mode is 1 (the default)
    // then all the geoms in that space are automatically destroyed as well.
    dSpaceDestroy(Space);

    // Destroy the world and everything in it. This includes all bodies and all joints that are not part of a joint group.
    dWorldDestroy(World);


}

void KidneyTask::SimLoopODE() {

    // dSpaceCollide determines which pairs of geoms in the space we pass to
    // it may potentially intersect. We must also pass the address of a
    // callback function that we will provide. The callback function is
    // responsible for determining which of the potential intersections are
    // actual collisions before adding the collision joints to our joint
    // group called contactgroup, this gives us the chance to set the
    // behaviour of these joints before adding them to the group. The second
    // parameter is a pointer to any data that we may want to pass to our
    // callback routine. We will cover the details of the nearCallback
    // routine in the next section.
    dSpaceCollide(Space, 0, &nearCallback);

    // Now we advance the simulation by calling dWorldQuickStep. This is a faster version of dWorldStep but it is also
    // slightly less accurate. As well as the World object ID we also pass a step size value. In each step the simulation
    // is updated by a certain number of smaller steps or iterations. The default number of iterations is 20 but you can
    // change this by calling dWorldSetQuickStepNumIterations.

    dWorldQuickStep(World, 0.005);

    // Remove all temporary collision joints now that the world has been stepped
    dJointGroupEmpty(contactgroup);

    // And we finish by calling DrawGeom which renders the objects according
//    // to their type or class
//    float pos;
//    float R;
    DrawGeom(Objct.Geom[0], 0, 0, 0);
//    pos = *dGeomGetPosition(Object.Geom[0]);
//
//    // If there was no rotation matrix given then get the existing rotation.
//    R = *dGeomGetRotation(Object.Geom[0]);



}


void nearCallback(void *data, dGeomID o1, dGeomID o2) {
    // Temporary index for each contact

    int i;

    // Get the dynamics body for each geom
    dBodyID b1 = dGeomGetBody(o1);

    dBodyID b2 = dGeomGetBody(o2);

    // Create an array of dContact objects to hold the contact joints
    dContact contact[MAX_CONTACTS];

    // Now we set the joint properties of each contact. Going into the full
    // details here would require a tutorial of its own. I'll just say that
    // the members of the dContact structure control the joint behaviour,
    // such as friction, velocity and bounciness. See section 7.3.7 of the ODE
    // manual and have fun experimenting to learn more.

    for (i = 0; i < MAX_CONTACTS; i++) {

        contact[i].surface.mode = dContactBounce | dContactSoftCFM;

        contact[i].surface.mu = dInfinity;

        contact[i].surface.mu2 = 0;

        contact[i].surface.bounce = 0.8;

        contact[i].surface.bounce_vel = 0.1;

        contact[i].surface.soft_cfm = 0.001;
    }

    // Here we do the actual collision test by calling dCollide. It returns
    // the number of actual contact points or zero if there were none. As
    // well as the geom IDs, max number of contacts we also pass the address
    // of a dContactGeom as the fourth parameter. dContactGeom is a
    // substructure of a dContact object so we simply pass the address of the
    // first dContactGeom from our array of dContact objects and then pass
    // the offset to the next dContactGeom as the fifth paramater, which is
    // the size of a dContact structure. That made sense didn't it?
    KidneyTask* self = static_cast<KidneyTask*>(data);

    if (int numc = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof
            (dContact))) {

        // To add each contact point found to our joint group we call
        // dJointCreateContact which is just one of the many
        // different joint types available.
        for (i = 0; i < numc; i++){
            // dJointCreateContact needs to know which world and joint group
            // to work with as well as the dContact object itself. It returns
            // a new dJointID which we then use with dJointAttach to finally
            // create the temporary contact joint between the two geom bodies.
            dJointID c = dJointCreateContact(World, contactgroup,
                                             contact + i);

            dJointAttach(c, b1, b2);
        }
    }
}

void KidneyTask::DrawGeom(dGeomID g, const dReal *pos, const dReal *R,
                          int show_aabb) {

    // If the geom ID is missing then return immediately.
    if (!g)
        return;

    // If there was no position vector supplied then get the existing position.
    if (!pos)
        pos = dGeomGetPosition(g);

    // If there was no rotation matrix given then get the existing rotation.
    if (!R)
        R = dGeomGetRotation(g);


    d_cube_actor->SetPosition(pos[0],pos[1],pos[2]);

//    // Get the geom's class type.
//    int type = dGeomGetClass (g);
//
//    if (type == dBoxClass){
//
//        // Create a temporary array of floats to hold the box dimensions.
//        dReal sides[3];
//        dGeomBoxGetLengths(g, sides);
//
//        // Now to actually render the box we make a call to DrawBox, passing
//        // the geoms dimensions, position vector and rotation matrix. And
//        // this function is the subject of our next discussion.
//        DrawBox(sides, pos, R);
//
//    }

}

//KidneyTask::~KidneyTask() {
//    CloseODE();
//}

