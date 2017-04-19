//
// Created by nima on 4/18/17.
//

#include <utils/Conversions.hpp>
#include "BuzzWireTask.h"

BuzzWireTask::BuzzWireTask(const double ring_radius, const double wire_radius) :
        ring_radius_(ring_radius), wire_radius_(wire_radius)
{


    sphereSource= vtkSmartPointer<vtkSphereSource>::New();

    sphere_translation = vtkSmartPointer<vtkTransform>::New();
    transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereActor = vtkSmartPointer<vtkActor>::New();

    parametricObject = vtkSmartPointer<vtkParametricTorus>::New();
    parametricFunctionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    ring_local_transform = vtkSmartPointer<vtkTransform>::New();
    ring_local_transform_filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    ring_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    ring_actor = vtkSmartPointer<vtkActor>::New();

    task_coordinate_axes = vtkSmartPointer<vtkAxesActor>::New();

    tool_current_frame_axes = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes = vtkSmartPointer<vtkAxesActor>::New();

    reader = vtkSmartPointer<vtkSTLReader>::New();
    mesh_transform = vtkSmartPointer<vtkTransform>::New();
    mesh_transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    mesh_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mesh_actor = vtkSmartPointer<vtkActor>::New();
    cellLocator = vtkSmartPointer<vtkCellLocator>::New();

    lineSource = vtkSmartPointer<vtkLineSource>::New();
    line_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();

    tool_current_pose =vtkSmartPointer<vtkMatrix4x4>::New();
//    tool_desired_pose =vtkSmartPointer<vtkMatrix4x4>::New();


    // --------------------------------------------------
    // Sphere

    sphereSource->SetRadius(0.003);
    sphereSource->SetThetaResolution(20);
    sphereSource->SetPhiResolution(20);
    sphereSource->Update();

    // to transform the data

    transformFilter->SetInputConnection(sphereSource->GetOutputPort());
    transformFilter->SetTransform(sphere_translation);
    transformFilter->Update();


    sphereMapper->SetInputConnection(transformFilter->GetOutputPort());

    int counter = 0;
    sphereActor->SetPosition(0.012, 0.00, 0.0);
    sphereActor->SetMapper(sphereMapper);
    sphereActor->GetProperty()->SetColor(0.8, 0.2, 0.4);


    // --------------------------------------------------
    // RING
    double ring_cross_section_radius = 0.001;
    double ring_scale = 0.006;
    parametricObject->SetCrossSectionRadius(ring_cross_section_radius/ ring_scale);
    parametricObject->SetRingRadius(ring_radius_/ ring_scale);

    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();


    // to transform the data

    ring_local_transform_filter->SetInputConnection(parametricFunctionSource->GetOutputPort());
    ring_local_transform->RotateX(90);
    ring_local_transform->Translate(0.0, ring_radius_/ring_scale, 0.0);

    ring_local_transform_filter->SetTransform(ring_local_transform);
    ring_local_transform_filter->Update();


    ring_mapper->SetInputConnection(ring_local_transform_filter->GetOutputPort());

    // Create an line_actor for the contours

    ring_actor->SetMapper(ring_mapper);
    ring_actor->SetScale(ring_scale);
    ring_actor->GetProperty()->SetColor(0.2, 0.4, 0.4);

    // --------------------------------------------------
    // FRAMES


    // The task_coordinate_axes are positioned with a user transform
//    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//    transform->Translate(0.0, 0, 0.0);
//    transform->RotateX(0);
//    task_coordinate_axes->SetUserTransform(transform);
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
    // MESH
    double tube_radius = 0.002;
    std::string inputFilename = "/home/charm/Desktop/cads/task1_3_mq.STL";
    reader->SetFileName(inputFilename.c_str());
    reader->Update();
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->DeepCopy(reader->GetOutput());

//    // Genreate Normals
//    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
//
//    normalGenerator->SetInputData(polydata);
//    normalGenerator->ComputePointNormalsOff();
//    normalGenerator->ComputeCellNormalsOn();
//    normalGenerator->Update();
//    /*
//    // Optional settings
//    normalGenerator->SetFeatureAngle(0.1);
//    normalGenerator->SetSplitting(1);
//    normalGenerator->SetConsistency(0);
//    normalGenerator->SetAutoOrientNormals(0);
//    normalGenerator->SetComputePointNormals(1);
//    normalGenerator->SetComputeCellNormals(0);
//    normalGenerator->SetFlipNormals(0);
//    normalGenerator->SetNonManifoldTraversal(1);
//    */
//    polydata = normalGenerator->GetOutput();
////    vtkSmartPointer<vtkDataArray>  cellNormals = vtkSmartPointer<vtkDataArray>::New();
////    cellNormals = polydata->GetCellData()->GetNormals();

    // transform
    mesh_transform->Translate(0.07, 0.03, 0.0);
    mesh_transform->RotateX(180);
    mesh_transform->RotateZ(150);


    mesh_transformFilter->SetInputConnection(reader->GetOutputPort());
    mesh_transformFilter->SetTransform(mesh_transform);
    mesh_transformFilter->Update();


    mesh_mapper->SetInputConnection(mesh_transformFilter->GetOutputPort());
//    std::cout << "reader: " << mesh_mapper->GetLength() << std::endl;
    mesh_actor->SetMapper(mesh_mapper);
//    mesh_actor->SetPosition(0.00, 0.00, 0.0);
//    mesh_actor->SetScale(0.0005);
//    mesh_actor->SetOrientation(90, 0.0 , 90.0);
    mesh_actor->GetProperty()->SetColor(0.7, 0.5, 0.2);
    mesh_actor->GetProperty()->SetSpecular(0.8);
    // --------------------------------------------------
    // CLOSEST POINT
    // Create the tree

//    cellLocator->SetDataSet(sphereActor->GetMapper()->GetInput());
    cellLocator->SetDataSet(mesh_transformFilter->GetOutput());
    cellLocator->BuildLocator();



    // Visualize

    line_mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> line_actor =
            vtkSmartPointer<vtkActor>::New();
    line_actor->SetMapper(line_mapper);
    line_actor->GetProperty()->SetLineWidth(4);


    actors.push_back(task_coordinate_axes);
    actors.push_back(tool_current_frame_axes);
    actors.push_back(tool_desired_frame_axes);
    actors.push_back(sphereActor);
    actors.push_back(mesh_actor);
    actors.push_back(ring_actor);
    actors.push_back(line_actor);

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

    FindClosestPoints();

    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose =vtkSmartPointer<vtkMatrix4x4>::New();
    VTKConversions::KDLFrameToVTKMatrix(tool_desired_pose_kdl, tool_desired_pose);
    tool_desired_frame_axes->SetUserMatrix(tool_desired_pose);

    counter++;
//            sphereActor->SetPosition(0.012 + 0.05 * sin(double(counter)/100*M_PI), 0.00, 0.0);
//            sphereActor->Modified();
    double dx = 0.05 * sin(double(counter)/100*M_PI);
    sphere_translation->Translate(dx/100, 0.00, 0.0);

//            transformFilter->SetTransform(sphere_translation);
//            transformFilter->Update();
//            sphereActor->Modified();


}


//----------------------------------------------------------------------------
void BuzzWireTask::FindClosestPoints() {

    double closestPointDist2; //the squared distance to the closest point will be returned here
    vtkIdType cell_id; //the cell id of the cell containing the closest point will be returned here
    int subId; //this is rarely used (in triangle strips only, I believe)

    double tool_point[3] = {tool_current_pose_kdl.p[0],
                            tool_current_pose_kdl.p[1],
                            tool_current_pose_kdl.p[2]};


    KDL::Vector radial_tool_point_kdl =  tool_current_pose_kdl * KDL::Vector(ring_radius_, 0.0, ring_radius_);
    double radial_tool_point[3] = {radial_tool_point_kdl[0],
                                   radial_tool_point_kdl[1],
                                   radial_tool_point_kdl[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};

    //Find the closest cell to the radial tool point
    cellLocator->Update();
    cellLocator->FindClosestPoint(tool_point, closest_point, cell_id, subId, closestPointDist2);
    closest_point_to_tool_point = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    //Find the closest cell to the radial tool point
    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_tool_point, closest_point, cell_id, subId, closestPointDist2);
    closest_point_to_radial_point = KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    // this will be calculated at a higher frequency in the main, here we do it once to update the actors
    // according to the new desired poses. may be removed later
    CalculatedDesiredToolPose(tool_current_pose_kdl, closest_point_to_tool_point, closest_point_to_radial_point, tool_desired_pose_kdl);

    lineSource->SetPoint1(tool_point);
    lineSource->SetPoint2(tool_desired_pose_kdl.p[0], tool_desired_pose_kdl.p[1], tool_desired_pose_kdl.p[2]);
    lineSource->Update();}


//----------------------------------------------------------------------------
KDL::Frame BuzzWireTask::GetDesiredToolPose() {

    CalculatedDesiredToolPose(tool_current_pose_kdl, closest_point_to_tool_point, closest_point_to_radial_point, tool_desired_pose_kdl);

    return tool_desired_pose_kdl;
}


//----------------------------------------------------------------------------
void BuzzWireTask::CalculatedDesiredToolPose(const KDL::Frame current_pose,
                                             const KDL::Vector closest_point_to_tool_point,
                                             const KDL::Vector closest_point_to_radial_points,
                                             KDL::Frame &desired_pose) {

    // find desired orientation
    KDL::Vector tool_to_tpcp = closest_point_to_tool_point - current_pose.p;

    // desired pose only when the ring is "around the wire" if it is too far we don't want fixtures
    if (tool_to_tpcp.Norm() < ring_radius_) {
        desired_pose.p = closest_point_to_tool_point
                         - (ring_radius_ - wire_radius_) *
                           (tool_to_tpcp / tool_to_tpcp.Norm());

        KDL::Vector radial_tool_point_kdl = tool_current_pose_kdl *
                                            KDL::Vector(ring_radius_, 0.0,
                                                        ring_radius_);

        KDL::Vector radial_to_rpcp =
                closest_point_to_radial_points - radial_tool_point_kdl;

        KDL::Vector desired_z, desired_y, desired_x;

        desired_z = tool_to_tpcp / tool_to_tpcp.Norm();
        desired_x = -radial_to_rpcp / radial_to_rpcp.Norm();
        desired_y = desired_z * desired_x;

        // make sure axes are perpendicular and normal
        desired_y = desired_y / desired_y.Norm();
        desired_x = desired_y * desired_z;
        desired_x = desired_x / desired_x.Norm();
        desired_z = desired_x * desired_y;
        desired_z = desired_z / desired_z.Norm();

        desired_pose.M = KDL::Rotation(desired_x, desired_y, desired_z);
    }
    else
    {
        desired_pose = current_pose;

    }
//    // Assuming that the normal to the ring is the z vector of current_orientation
//    KDL::Vector ring_normal = current_pose.M.UnitZ();
//
//    // find the normal vector for the rotation to desired pose
//    KDL::Vector rotation_axis = ring_normal * tool_to_tpcp;
//
//    // find the magnitude of rotation
//    // we are interested in the smallest rotation (in the same quarter)
//    double rotation_angle = acos( (KDL::dot(ring_normal, tool_to_tpcp)) /
//                                  (ring_normal.Norm() * tool_to_tpcp.Norm()) );
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
