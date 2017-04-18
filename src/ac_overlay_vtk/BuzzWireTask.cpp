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

    double tool_point[3] = {tool_current_pose->Element[0][3],
                            tool_current_pose->Element[1][3],
                            tool_current_pose->Element[2][3],};
    ; //the coordinates of the closest point will be returned here
    double closestPointDist2; //the squared distance to the closest point will be returned here
    vtkIdType cell_id; //the cell id of the cell containing the closest point will be returned here
    int subId; //this is rarely used (in triangle strips only, I believe)

    cellLocator->Update();
    cellLocator->FindClosestPoint(tool_point, closest_point, cell_id, subId, closestPointDist2);

    lineSource->SetPoint1(tool_point);
    lineSource->SetPoint2(closest_point);
    lineSource->Update();}


//----------------------------------------------------------------------------
KDL::Frame BuzzWireTask::GetDesiredToolPose() {

    CalculatedDesiredToolPose(tool_current_pose_kdl, closest_point, tool_desired_pose_kdl);

    return tool_desired_pose_kdl;
}

//----------------------------------------------------------------------------
void BuzzWireTask::CalculatedDesiredToolPose(const KDL::Frame current_pose, double closest_points[],
                                             KDL::Frame &desired_pose) {

    desired_pose.p = KDL::Vector(closest_points[0], closest_points[1], closest_points[2]);

    // find desired orientation
    KDL::Vector ac_path_tangent_current = KDL::Vector(closest_points[0] - current_pose.p[0],
                                                      closest_points[1] - current_pose.p[1],
                                                      closest_points[2] - current_pose.p[2]);


    // Assuming that the normal to the ring is the z vector of current_orientation
    KDL::Vector ring_normal = current_pose.M.UnitZ();

    // find the normal vector for the rotation to desired pose
    KDL::Vector rotation_axis = ring_normal * ac_path_tangent_current;

    // find the magnitude of rotation
    // we are interested in the smallest rotation (in the same quarter)
    double rotation_angle = acos( (KDL::dot(ring_normal, ac_path_tangent_current)) /
                                  (ring_normal.Norm() * ac_path_tangent_current.Norm()) );

    // If the ring is perpendicular to the tangent switch the direction of the tangent
    // to get the smaller rotation
    if(rotation_angle > M_PI/2)
        rotation_angle = rotation_angle - M_PI;

    KDL::Rotation rotation_to_desired;
    conversions::AxisAngleToKDLRotation(rotation_axis, rotation_angle, rotation_to_desired);


    desired_pose.M = rotation_to_desired * current_pose.M ;

}
