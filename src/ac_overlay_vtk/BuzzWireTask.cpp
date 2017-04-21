//
// Created by nima on 4/18/17.
//

#include <utils/Conversions.hpp>
#include <vtkCubeSource.h>
#include "BuzzWireTask.h"

BuzzWireTask::BuzzWireTask(const double ring_radius, const bool show_ref_frames)
        :
        ring_radius_(ring_radius),
        show_ref_frames_(show_ref_frames)
{

    // --------------------------------------------------
    //  ACTIVE CONSTRAINT
    // --------------------------------------------------
    // these parameters could be set as ros parameters but since
    // they change during the task we are hard coding them here.
    ac_parameters.method =                  0; // 0 for visco/elastic
    ac_parameters.active =                  1; // 0 for visco/elastic

    ac_parameters.max_force =               4.0;
    ac_parameters.linear_elastic_coeff =    1000.0;
    ac_parameters.linear_damping_coeff =    10.0;

    ac_parameters.max_torque =              0.03;
    ac_parameters.angular_elastic_coeff =   0.04;
    ac_parameters.angular_damping_coeff =   0.002;


    // --------------------------------------------------
    //  INITIALIZING GRAPHICS ACTORS
    // --------------------------------------------------

    ring_actor = vtkSmartPointer<vtkActor>::New();
    error_sphere_actor = vtkSmartPointer<vtkActor>::New();

    task_coordinate_axes = vtkSmartPointer<vtkAxesActor>::New();

    tool_current_frame_axes = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes = vtkSmartPointer<vtkAxesActor>::New();

    cellLocator = vtkSmartPointer<vtkCellLocator>::New();

    line1_source = vtkSmartPointer<vtkLineSource>::New();

    line2_source = vtkSmartPointer<vtkLineSource>::New();

    tool_current_pose =vtkSmartPointer<vtkMatrix4x4>::New();

    // --------------------------------------------------
    // RING
    double ring_cross_section_radius = 0.0005;
    double ring_scale = 0.006;

    vtkSmartPointer<vtkParametricTorus> parametricObject =
            vtkSmartPointer<vtkParametricTorus>::New();
    parametricObject->SetCrossSectionRadius(ring_cross_section_radius/ ring_scale);
    parametricObject->SetRingRadius(ring_radius_/ ring_scale);

    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
            vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();

    // to transform the data
    vtkSmartPointer<vtkTransformPolyDataFilter> ring_local_transform_filter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_local_transform_filter->SetInputConnection(parametricFunctionSource->GetOutputPort());

    vtkSmartPointer<vtkTransform> ring_local_transform =
            vtkSmartPointer<vtkTransform>::New();
    ring_local_transform->RotateX(90);
    ring_local_transform->Translate(0.0, ring_radius_/ring_scale, 0.0);

    ring_local_transform_filter->SetTransform(ring_local_transform);
    ring_local_transform_filter->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_mapper->SetInputConnection(ring_local_transform_filter->GetOutputPort());

    ring_actor->SetMapper(ring_mapper);
    ring_actor->SetScale(ring_scale);
    ring_actor->GetProperty()->SetColor(0.1, 0.3, 0.4);
    ring_actor->GetProperty()->SetSpecular(0.7);

    // --------------------------------------------------
    // FRAMES


    // The task_coordinate_axes are positioned with a user transform
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



    // --------------------------------------------------
    // Stand MESH hq
    std::string  inputFilename = "/home/charm/Desktop/cads/task1_4_stand.STL";
    vtkSmartPointer<vtkSTLReader> stand_mesh_reader = vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    stand_mesh_reader->SetFileName(inputFilename.c_str());
    stand_mesh_reader->Update();

    // transform
    vtkSmartPointer<vtkTransform> stand_transform = vtkSmartPointer<vtkTransform>::New();
    stand_transform->Translate(0.07, 0.04, 0.025);
    stand_transform->RotateX(180);
    stand_transform->RotateZ(150);

    vtkSmartPointer<vtkTransformPolyDataFilter> stand_mesh_transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    stand_mesh_transformFilter->SetInputConnection(stand_mesh_reader->GetOutputPort());
    stand_mesh_transformFilter->SetTransform(stand_transform);
    stand_mesh_transformFilter->Update();


    vtkSmartPointer<vtkPolyDataMapper> stand_mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    stand_mesh_mapper->SetInputConnection(stand_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> stand_mesh_actor = vtkSmartPointer<vtkActor>::New();
    stand_mesh_actor->SetMapper(stand_mesh_mapper);
    stand_mesh_actor->GetProperty()->SetColor(0.45, 0.4, 0.4);
//    stand_mesh_actor->GetProperty()->SetSpecular(0.8);



    // --------------------------------------------------
    // MESH hq is for rendering and lq is for finding generating active constraints
    inputFilename = "/home/charm/Desktop/cads/task1_4_tube.STL";
    vtkSmartPointer<vtkSTLReader> hq_mesh_reader = vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    hq_mesh_reader->SetFileName(inputFilename.c_str());
    hq_mesh_reader->Update();
    vtkSmartPointer<vtkTransform> tube_transform = vtkSmartPointer<vtkTransform>::New();
    tube_transform->DeepCopy(stand_transform);
    tube_transform->RotateX(-15);

    vtkSmartPointer<vtkTransformPolyDataFilter> hq_mesh_transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    hq_mesh_transformFilter->SetInputConnection(hq_mesh_reader->GetOutputPort());
    hq_mesh_transformFilter->SetTransform(tube_transform);
    hq_mesh_transformFilter->Update();


    vtkSmartPointer<vtkPolyDataMapper> hq_mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    hq_mesh_mapper->SetInputConnection(hq_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> hq_mesh_actor = vtkSmartPointer<vtkActor>::New();
    hq_mesh_actor->SetMapper(hq_mesh_mapper);
    hq_mesh_actor->GetProperty()->SetColor(0.9, 0.4, 0.1);
    hq_mesh_actor->GetProperty()->SetSpecular(0.8);
    hq_mesh_actor->GetProperty()->SetSpecularPower(80);
//    hq_mesh_actor->GetProperty()->SetOpacity(0.5);


    // --------------------------------------------------
    // MESH hq is for rendering and lq is for finding generating active constraints
    inputFilename = "/home/charm/Desktop/cads/super_ring_lq.STL";
    vtkSmartPointer<vtkSTLReader> ring_guides_mesh_reader = vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    ring_guides_mesh_reader->SetFileName(inputFilename.c_str());
    ring_guides_mesh_reader->Update();



    // transform
    vtkSmartPointer<vtkTransform> ring_transform = vtkSmartPointer<vtkTransform>::New();
    ring_transform->Translate(0.0, 0.0, 0.0035);
    ring_transform->RotateX(90);

    vtkSmartPointer<vtkTransformPolyDataFilter> ring_guides_mesh_transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_guides_mesh_transformFilter->SetInputConnection(ring_guides_mesh_reader->GetOutputPort());
    ring_guides_mesh_transformFilter->SetTransform(ring_transform);
    ring_guides_mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> ring_guides_mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_guides_mesh_mapper->SetInputConnection(ring_guides_mesh_transformFilter->GetOutputPort());

    ring_guides_mesh_actor = vtkSmartPointer<vtkActor>::New();
    ring_guides_mesh_actor->SetMapper(ring_guides_mesh_mapper);
    ring_guides_mesh_actor->GetProperty()->SetColor(1.0, 1.0, 1.0);
    ring_guides_mesh_actor->GetProperty()->SetOpacity(0.5);
//    hq_mesh_actor->GetProperty()->SetOpacity(0.5);


    // --------------------------------------------------
    // MESH lq
    inputFilename = "/home/charm/Desktop/cads/task1_4_wire.STL";
    vtkSmartPointer<vtkSTLReader> lq_mesh_reader = vtkSmartPointer<vtkSTLReader>::New();
    std::cout << "Loading stl file from: " << inputFilename << std::endl;
    lq_mesh_reader->SetFileName(inputFilename.c_str());
    lq_mesh_reader->Update();


    vtkSmartPointer<vtkTransformPolyDataFilter> lq_mesh_transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    lq_mesh_transformFilter->SetInputConnection(lq_mesh_reader->GetOutputPort());
    lq_mesh_transformFilter->SetTransform(tube_transform);
    lq_mesh_transformFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> lq_mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    lq_mesh_mapper->SetInputConnection(lq_mesh_transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> lq_mesh_actor = vtkSmartPointer<vtkActor>::New();
    lq_mesh_actor->SetMapper(lq_mesh_mapper);

    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(lq_mesh_transformFilter->GetOutput());
    cellLocator->BuildLocator();

    // --------------------------------------------------
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
    floor_actor->SetPosition(floor_dimensions[0]/2, floor_dimensions[1]/2 , -floor_dimensions[2]);
    floor_actor->GetProperty()->SetOpacity(0.3);
    floor_actor->GetProperty()->SetColor(0.7, 0.7, 0.7);

    //    floor_actor->SetScale(ring_scale);
    // Lines
    vtkSmartPointer<vtkPolyDataMapper> line1_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    vtkSmartPointer<vtkActor> line1_actor =
            vtkSmartPointer<vtkActor>::New();
    line1_actor->SetMapper(line1_mapper);
    line1_actor->GetProperty()->SetLineWidth(5);
    line1_actor->GetProperty()->SetColor(1, 0, 1);

    vtkSmartPointer<vtkPolyDataMapper> line2_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    line2_mapper->SetInputConnection(line2_source->GetOutputPort());
    vtkSmartPointer<vtkActor> line2_actor =
            vtkSmartPointer<vtkActor>::New();
    line2_actor->SetMapper(line2_mapper);
    line2_actor->GetProperty()->SetLineWidth(3);


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



    // --------------------------------------------------
    // Error sphere
    vtkSmartPointer<vtkSphereSource> sphereSource =
            vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetCenter(-0.02, 0.08, 0.01);
    sphereSource->SetRadius(0.005);
    sphereSource->SetPhiResolution(10);
    sphereSource->SetThetaResolution(10);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(sphereSource->GetOutputPort());
    error_sphere_actor->SetMapper(sphere_mapper);
    error_sphere_actor->GetProperty()->SetColor(0.0, 0.7, 0.1);

    if(show_ref_frames_) {
        actors.push_back(task_coordinate_axes);
        actors.push_back(tool_current_frame_axes);
        actors.push_back(tool_desired_frame_axes);
    }

    actors.push_back(hq_mesh_actor);
    actors.push_back(stand_mesh_actor);
//    actors.push_back(floor_actor);
    //    actors.push_back(lq_mesh_actor);
    actors.push_back(ring_actor);
    actors.push_back(line1_actor);
//    actors.push_back(line2_actor);
    actors.push_back(ring_guides_mesh_actor);
    actors.push_back(error_sphere_actor);
    // add the annotation as the last actor
//    actors.push_back(cornerAnnotation);


}

//----------------------------------------------------------------------------
std::vector<vtkSmartPointer<vtkProp> > BuzzWireTask::GetActors() {
    return actors;
}

//----------------------------------------------------------------------------
void BuzzWireTask::SetCurrentToolPose(const KDL::Frame &tool_pose) {

    tool_current_pose_kdl = tool_pose;
    VTKConversions::KDLFrameToVTKMatrix(tool_pose, tool_current_pose);

}

//----------------------------------------------------------------------------
void BuzzWireTask::UpdateActors() {

    tool_current_frame_axes->SetUserMatrix(tool_current_pose);

    ring_actor->SetUserMatrix(tool_current_pose);
    ring_guides_mesh_actor->SetUserMatrix(tool_current_pose);

    FindClosestPoints();

    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose =vtkSmartPointer<vtkMatrix4x4>::New();
    VTKConversions::KDLFrameToVTKMatrix(tool_desired_pose_kdl, tool_desired_pose);
    tool_desired_frame_axes->SetUserMatrix(tool_desired_pose);

//    std::stringstream error_message;
//    error_message << error_position;
//    cornerAnnotation->SetText( 1, error_message.str().c_str() );
    double max_error = 0.004;
    double error_ratio = error_position / max_error;
    if(error_ratio>1.0)
        error_ratio = 1.0;

    error_sphere_actor->GetProperty()->SetColor(error_ratio, 1 - error_ratio, 0.1);

}


//----------------------------------------------------------------------------
void BuzzWireTask::FindClosestPoints() {

    double closestPointDist2; //the squared distance to the closest point will be returned here
    vtkIdType cell_id; //the cell id of the cell containing the closest point will be returned here
    int subId; //this is rarely used (in triangle strips only, I believe)


    //Find the closest cell to the grip point
    double grip_point[3] = {tool_current_pose_kdl.p[0],
                            tool_current_pose_kdl.p[1],
                            tool_current_pose_kdl.p[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};
    cellLocator->Update();
    cellLocator->FindClosestPoint(grip_point, closest_point, cell_id, subId, closestPointDist2);
    closest_point_to_grip_point = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);


    //Find the closest cell to the the central point
    KDL::Vector ring_centeral_point_kdl =  tool_current_pose_kdl * KDL::Vector(0.0, 0.0, ring_radius_);
    double ring_central_point[3] = {ring_centeral_point_kdl[0],
                                    ring_centeral_point_kdl[1],
                                    ring_centeral_point_kdl[2]};
    cellLocator->Update();
    cellLocator->FindClosestPoint(ring_central_point, closest_point, cell_id, subId, closestPointDist2);
    closest_point_to_ring_center = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    //Find the closest cell to the radial tool point
    KDL::Vector radial_tool_point_kdl =  tool_current_pose_kdl * KDL::Vector(ring_radius_, 0.0, ring_radius_);
    double radial_tool_point[3] = {radial_tool_point_kdl[0],
                                   radial_tool_point_kdl[1],
                                   radial_tool_point_kdl[2]};
    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_tool_point, closest_point, cell_id, subId, closestPointDist2);
    closest_point_to_radial_point = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

//    // this will be calculated in the main, here we do it once to update the actors
//    // according to the new desired poses. may be removed later
//    CalculatedDesiredToolPose(tool_current_pose_kdl,
//                              closest_point_to_ring_center,
//                              closest_point_to_radial_point,
//                              closest_point_to_grip_point , tool_desired_pose_kdl);

    line1_source->SetPoint1(ring_central_point);
    line1_source->SetPoint2(tool_desired_pose_kdl.p[0], tool_desired_pose_kdl.p[1], tool_desired_pose_kdl.p[2]);
    line1_source->Update();


}


//----------------------------------------------------------------------------
KDL::Frame BuzzWireTask::GetDesiredToolPose() {



    CalculatedDesiredToolPose(tool_current_pose_kdl,
                              closest_point_to_ring_center,
                              closest_point_to_radial_point,
                              closest_point_to_grip_point,
                              tool_desired_pose_kdl);

    return tool_desired_pose_kdl;
}


//----------------------------------------------------------------------------
void BuzzWireTask::CalculatedDesiredToolPose(const KDL::Frame current_pose,
                                             const KDL::Vector closest_point_to_ring_center,
                                             const KDL::Vector closest_point_to_radial_point,
                                             const KDL::Vector closest_point_to_grip_point,
                                             KDL::Frame &desired_pose) {
    // NOTE: All the closest points are on the wire mesh

    // locate the center of the ring
    KDL::Vector ring_center = current_pose * KDL::Vector(0.0, 0.0, ring_radius_);
    // Find the vector from ring center to the corresponding closest point on the wire
    KDL::Vector ring_center_to_cp = closest_point_to_ring_center - ring_center;

    // desired pose only when the ring is close to the wire.if it is too
    // far we don't want fixtures
    if (ring_center_to_cp.Norm()  < 3*ring_radius_) {
        ac_parameters.active = 1;
        ac_params_changed = true;

        // Desired position is one that puts the center of the wire on the
        // center of the ring.

        //---------------------------------------------------------------------
        // Find the desired position
        KDL::Vector wire_center = ring_center + ring_center_to_cp;
        // Turns out trying to estimate the center of the wire makes things worse
        // so I removed the following term that was used in finding wire_center:
        // wire_radius_ * (ring_center_to_cp/ring_center_to_cp.Norm());

        error_position = (wire_center - ring_center).Norm();
        desired_pose.p = current_pose.p + wire_center - ring_center;

        //---------------------------------------------------------------------
        // Find the desired orientation
        // We use two vectors to estimate the tangent of the direction of the wire.
        // One is from the grip point (current tool tip) to its closest point on
        // the wire and the other is from a point on the side of the ring (90 deg
        // from the grip point). The estimated tangent is the cross product of these
        // two (after normalization). This is just a quick the non-ideal approximation.
        // Note that we could have used the central point instead of the tool point
        // but that vector gets pretty small and unstable when we're close to the
        // desired pose.
        KDL::Vector radial_point_kdl = current_pose * KDL::Vector(ring_radius_, 0.0,
                                                        ring_radius_);
        KDL::Vector radial_to_cp =
                closest_point_to_radial_point - radial_point_kdl;

        KDL::Vector desired_z, desired_y, desired_x;

        KDL::Vector grip_to_cp = closest_point_to_grip_point - current_pose.p;

        desired_z = grip_to_cp / grip_to_cp.Norm();
        desired_x = -radial_to_cp / radial_to_cp.Norm();
        desired_y = desired_z * desired_x;

        // make sure axes are perpendicular and normal
        desired_y = desired_y / desired_y.Norm();
        desired_x = desired_y * desired_z;
        desired_x = desired_x / desired_x.Norm();
        desired_z = desired_x * desired_y;
        desired_z = desired_z / desired_z.Norm();

         desired_pose.M = KDL::Rotation(desired_x, desired_y, desired_z);
        //        desired_pose.M = current_pose.M;
    }
    else
    {
        desired_pose = current_pose;
        ac_parameters.active = 0;
        ac_params_changed = true;

    }
//    // Assuming that the normal to the ring is the z vector of current_orientation
//    KDL::Vector ring_normal = current_pose.M.UnitZ();
//
//    // find the normal vector for the rotation to desired pose
//    KDL::Vector rotation_axis = ring_normal * ring_center_to_cp;
//
//    // find the magnitude of rotation
//    // we are interested in the smallest rotation (in the same quarter)
//    double rotation_angle = acos( (KDL::dot(ring_normal, ring_center_to_cp)) /
//                                  (ring_normal.Norm() * ring_center_to_cp.Norm()) );
//
//    // If the ring is perpendicular to the tangent switch the direction of the tangent
//    // to get the smaller rotation
//    if(rotation_angle > M_PI/2)
//        rotation_angle = rotation_angle - M_PI;
//
//    KDL::Rotation rotation_to_desired;
//    conversions::AxisAngleToKDLRotation(rotation_axis, rotation_angle, rotation_to_desired);
//
//
//    desired_pose.M = rotation_to_desired * current_pose.M ;

}

bool BuzzWireTask::IsACParamChanged() {
    return ac_params_changed;
}

active_constraints::ActiveConstraintParameters BuzzWireTask::GetACParameters() {

    ac_params_changed =false; // assuming once we read it we can consider it unchanged
    return ac_parameters;
}

