//
// Created by nima on 4/18/17.
//

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <vtkConeSource.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_conversions/kdl_msg.h>
#include "TaskSteadyHand.h"
#include <kdl/frames.hpp>
#include <vtkOBJReader.h>

namespace SHColors {
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

TaskSteadyHand::TaskSteadyHand(
        const std::string stl_file_dir,
        const bool show_ref_frames,
        const bool biman,
        const bool with_guidance,
        const double haptic_loop_rate,
        const std::string slave_names_in[],
        KDL::Frame *slave_to_world_tr
    )
    :
    VTKTask(show_ref_frames, biman, with_guidance, haptic_loop_rate),
    stl_files_dir(stl_file_dir),
    slave_frame_to_world_frame_tr(slave_to_world_tr),
//        show_ref_frames(show_ref_frames),
//        bimanual(biman),
//        with_guidance(with_guidance),
    destination_ring_counter(0),
    ac_params_changed(true),
    task_state(SHTaskState::Idle),
    number_of_repetition(0),
    n_score_history(10),
    time_last(ros::Time::now())
{

    InitBullet();





    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject *floor = new BulletVTKObject(
        ObjectShape::STATICPLANE, ObjectType::DYNAMIC,
        floor_dims, dummy_pose, 0.0, NULL
    );
    dynamics_world->addRigidBody(floor->GetBody());


    BulletVTKObject *board;
    // -----------------------
    // -------------------------------------------------------------------------
    // Create a cube for the board

    {
        double friction = 0.005;
        double board_dimensions[3]  = {0.14, 0.12, 0.01};

        double pose[7]{
            board_dimensions[0] / 2.45, board_dimensions[1] / 2.78,
            -board_dimensions[2] / 2, 0, 0, 0, 1
        };

        std::vector<double> dim = {
            board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose, 0.0, NULL,
            friction
        );
        //    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.5, 0.3, 0.1);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
    }




    slave_names = new std::string[bimanual+1];

    *slave_names = *slave_names_in;

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

    tool_current_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_current_pose[0] = vtkSmartPointer<vtkMatrix4x4>::New();

    if(bimanual){
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

    }



    // -------------------------------------------------------------------------
    //// Create ring mesh

    std::vector<double> _dim = {0.002};

    float friction = 50;
    float density = 50000; // kg/m3
    std::stringstream input_file_dir;
    //input_file_dir << stl_file_dir << std::string("task_steady_hand_ring.obj");
    //std::string mesh_file_dir_str = input_file_dir.str();
    std::string mesh_file_dir_str;

    KDL::Rotation cyl_rot(KDL::Rotation::RotZ(30.0/180.0*M_PI)
                              //*KDL::Rotation::RotX(M_PI/2)
                              *KDL::Rotation::RotZ(M_PI/2));
    double cx, cy,cz, cw;
    cyl_rot.GetQuaternion(cx, cy, cz, cw);

    for (int l = 0; l < 1 + (int) bimanual; ++l) {

        double pose[7]{0.025-l*0.01, 0.034-l*0.008, 0.025, cx, cy, cz, cw};

        input_file_dir.str("");
        input_file_dir << stl_file_dir << std::string("task_steady_hand_ring0")
                       << std::string(".obj");
        mesh_file_dir_str = input_file_dir.str();

        ring_mesh[l] = new
            BulletVTKObject(
            ObjectShape::MESH,
            ObjectType::DYNAMIC, _dim, pose, density,
            &mesh_file_dir_str,
            friction
        );

        dynamics_world->addRigidBody(ring_mesh[l]->GetBody());
        actors.push_back(ring_mesh[l]->GetActor());
        ring_mesh[l]->GetActor()->GetProperty()->SetColor(SHColors::Turquoise);
        ring_mesh[l]->GetActor()->GetProperty()->SetSpecular(0.7);

        ring_mesh[l]->GetBody()->setContactStiffnessAndDamping(2000, 100);
    }

    // -------------------------------------------------------------------------
    // Cylinder supporting the rings in starting position

    double pose_cyl[7]{0.02, 0.03, 0.02,
        cx, cy, cz, cw};

    _dim = {0.002, 0.04};
    friction = 0.005;

    supporting_cylinder = new
        BulletVTKObject(
        ObjectShape::CYLINDER,
        ObjectType::DYNAMIC, _dim, pose_cyl, 0.0,
        &mesh_file_dir_str,
        friction
    );

    dynamics_world->addRigidBody(supporting_cylinder->GetBody());
    actors.push_back(supporting_cylinder->GetActor());
    supporting_cylinder->GetActor()->GetProperty()->SetColor(SHColors::Orange);
    supporting_cylinder->GetActor()->GetProperty()->SetSpecular(0.7);


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
    destination_ring_actor->GetProperty()->SetColor(SHColors::Green);
    //destination_ring_actor->GetProperty()->SetOpacity(0.5);

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

    // hard coding the position of of the destinations
    // if the base is rotated the destinations will not be valid anymore...
    KDL::Vector base_position = KDL::Vector(0.095, 0.09, 0.025);
    //
    idle_point  = base_position + KDL::Vector(-0.056, -0.034, 0.004);
    start_point  = base_position + KDL::Vector(-0.048, -0.029, 0.005);
    end_point  = base_position + KDL::Vector(-0.016, -0.017, 0.028);

    //idle_point  = base_position + KDL::Vector(0.056, -0.032, 0.004);
    //start_point  = base_position + KDL::Vector(0.048, -0.026, 0.005);
    //end_point  = base_position + KDL::Vector(0.020, -0.003, 0.028);

    // -------------------------------------------------------------------------
    // Stand MESH hq
    input_file_dir.str("");
    input_file_dir << stl_files_dir << std::string("task1_4_stand.STL");

    vtkSmartPointer<vtkSTLReader> stand_mesh_reader =
        vtkSmartPointer<vtkSTLReader>::New();
    ROS_DEBUG("Loading stl file from: %s", input_file_dir.str().c_str());
    stand_mesh_reader->SetFileName(input_file_dir.str().c_str());
    stand_mesh_reader->Update();

    // transform
    vtkSmartPointer<vtkTransform> stand_transform =
        vtkSmartPointer<vtkTransform>::New();

    stand_transform->Translate(base_position[0], base_position[1],
                               base_position[2]);
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
    stand_mesh_actor->GetProperty()->SetColor(SHColors::Gray);
    //    stand_mesh_actor->GetProperty()->SetSpecular(0.8);


    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for generating
    // active constraints

    KDL::Rotation tube_rot(KDL::Rotation::RotZ(30.0/180.0*M_PI)
                               *KDL::Rotation::RotX(M_PI/2)
                               *KDL::Rotation::RotZ(M_PI));
    double qx, qy,qz, qw;
    tube_rot.GetQuaternion(qx, qy,qz, qw);
    double pose_tube[7]{base_position[0], base_position[1], base_position[2],
        qx, qy,qz, qw};

    _dim[0] = 0.002;
    input_file_dir.str("");
    input_file_dir << stl_files_dir << std::string("task_steady_hand_rollercoaster.obj");
    mesh_file_dir_str = input_file_dir.str();

    friction = 0.001;

    tube_mesh = new
        BulletVTKObject(
        ObjectShape::MESH,
        ObjectType::DYNAMIC, _dim, pose_tube, 0.0,
        &mesh_file_dir_str,
        friction
    );

    dynamics_world->addRigidBody(tube_mesh->GetBody());
    actors.push_back(tube_mesh->GetActor());
    tube_mesh->GetActor()->GetProperty()->SetColor(SHColors::Orange);
    tube_mesh->GetActor()->GetProperty()->SetSpecular(0.8);
    tube_mesh->GetActor()->GetProperty()->SetSpecularPower(80);


    vtkSmartPointer<vtkTransform> tube_transform =
        vtkSmartPointer<vtkTransform>::New();
    tube_transform->DeepCopy(stand_transform);

    vtkSmartPointer<vtkTransformPolyDataFilter> hq_mesh_transformFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();

        //tube_mesh_actor->GetProperty()->SetOpacity(0.5);


    // -------------------------------------------------------------------------
    // MESH lq
    input_file_dir.str("");
    input_file_dir << stl_files_dir << std::string
        ("task_steady_hand_rollercoaster_thin.obj");

    vtkSmartPointer<vtkOBJReader> lq_mesh_reader =
        vtkSmartPointer<vtkOBJReader>::New();
    ROS_DEBUG("Loading stl file from: %s", input_file_dir.str().c_str());
    lq_mesh_reader->SetFileName(input_file_dir.str().c_str());
    lq_mesh_reader->Update();


    vtkSmartPointer<vtkTransformPolyDataFilter> lq_mesh_transformFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    lq_mesh_transformFilter->SetInputConnection(
        lq_mesh_reader->GetOutputPort());
    // TODO: just changed the wire mesh and its coordinate frame doesn't match
    // that of the tube. Fixing it here temporarily
    vtkSmartPointer<vtkTransform> wire_transform =
        vtkSmartPointer<vtkTransform>::New();
    wire_transform->DeepCopy(tube_transform);
    wire_transform->Translate(0.0, 0.0, -0.030);

    lq_mesh_transformFilter->SetTransform(wire_transform);
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
    // Lines
    vtkSmartPointer<vtkPolyDataMapper> line1_mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    line1_actor->SetMapper(line1_mapper);
    line1_actor->GetProperty()->SetLineWidth(3);
//    line1_actor->GetProperty()->SetColor(SHColors::DeepPink);
    line1_actor->GetProperty()->SetOpacity(0.8);

    vtkSmartPointer<vtkPolyDataMapper> line2_mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    line2_mapper->SetInputConnection(line2_source->GetOutputPort());

    line2_actor->SetMapper(line2_mapper);
    line2_actor->GetProperty()->SetLineWidth(3);
//    line2_actor->GetProperty()->SetColor(SHColors::DeepPink);
    line2_actor->GetProperty()->SetOpacity(0.8);

    // -------------------------------------------------------------------------
    // Create kinematic jaw (gripper)
    //
    // The gripper is a 5 link mechanism, link 0 is a base link, 1st and 2nd
    // are angular jaws (like a scissor) so their orientation is related to
    // the gripper angle of the master, 3rd and 4th change position along one
    // axis according to the gripper angle, like a clamp. These last links
    // are used to generate enough normal force for a stable grasping, since
    // the scissor type links push the objects outwards.
    // make sure tu set a high friction coefficient for the objects you want
    // to grasp.
    // In addition, make sure you limit the gripper angle so that the user
    // can't press the object too much. Otherwise the injected energy would
    // be so high that no friction can compensate it.
    {
        float gripper_density = 0; // kg/m3
        float gripper_friction = 50;
        double gripper_pose[7]{0, 0, 0, 0, 0, 0, 1};
        gripper_link_dims =
            {{0.003, 0.003, 0.003}
             , {0.002, 0.001, 0.007}
             , {0.002, 0.001, 0.007}
             , {0.002, 0.002, 0.005}
             , {0.002, 0.002, 0.005}};

        for (int i = 0; i < 5; ++i) {

                right_gripper_links[i] =
                    new BulletVTKObject(
                        ObjectShape::BOX, ObjectType::KINEMATIC,
                        gripper_link_dims[i], gripper_pose,
                        gripper_density,
                        NULL, gripper_friction
                    );
            dynamics_world->addRigidBody(right_gripper_links[i]->GetBody());
            actors.push_back(right_gripper_links[i]->GetActor());
            right_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            right_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);

            right_gripper_links[i]->GetBody()->setContactStiffnessAndDamping
                (2000, 100);
        }


        for (int i = 0; i < 5; ++i) {
            left_gripper_links[i] =
                new BulletVTKObject(
                    ObjectShape::BOX, ObjectType::KINEMATIC,
                    gripper_link_dims[i], gripper_pose,
                    gripper_density,
                    NULL, gripper_friction
                );
            dynamics_world->addRigidBody(left_gripper_links[i]->GetBody());
            actors.push_back(left_gripper_links[i]->GetActor());
            left_gripper_links[i]->GetActor()->GetProperty()->SetColor(0.65f,0.7f,0.7f);
            left_gripper_links[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            left_gripper_links[i]->GetActor()->GetProperty()->SetSpecular(0.8);

            left_gripper_links[i]->GetBody()->setContactStiffnessAndDamping
                (2000, 100);
        }

        // Cylinders resembling the robotic arms

        KDL::Vector cam_position(0.118884, 0.27565, 0.14583);

        rcm[0] = {cam_position.x() - 0.1, cam_position.y(), cam_position.z()};
        rcm[1] = {cam_position.x() + 0.1, cam_position.y(), cam_position.z()};

        for (int i = 0; i < 1 + (int)bimanual; ++i) {

            std::vector<double> arm_dim = { 0.002, rcm[i].Norm()*2};

            arm[i] = new BulletVTKObject(
                ObjectShape::CYLINDER,
                ObjectType::KINEMATIC, arm_dim, gripper_pose, 0.0,
                NULL, gripper_friction
            );
            dynamics_world->addRigidBody(arm[i]->GetBody());
            actors.push_back(arm[i]->GetActor());
            arm[i]->GetActor()->GetProperty()->SetColor(1.0f,0.7f,0.7f);
            arm[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            arm[i]->GetActor()->GetProperty()->SetSpecular(0.8);
            arm[i]->GetActor()->GetProperty()->SetOpacity(1.0);

            arm[i]->GetBody()->setContactStiffnessAndDamping(2000, 100);
        }
    }

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
        actor->GetProperty()->SetColor(SHColors::Gray);
        actor->SetPosition(0.09- (double)i * 0.006, 0.132 - (double)i * 0.0003,
                           0.01);
        score_sphere_actors.push_back(actor);
    }


    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        actors.push_back(task_coordinate_axes);

        for (int k = 0; k < 1 + (int)bimanual; ++k) {
            actors.push_back(tool_current_frame_axes[k]);
            actors.push_back(tool_desired_frame_axes[k]);
        }
    }

    actors.push_back(stand_mesh_actor);
    //actors.push_back(lq_mesh_actor);
    if(bimanual){
        actors.push_back(line1_actor);
        actors.push_back(line2_actor);
    }
    actors.push_back(destination_ring_actor);
    for (int j = 0; j < score_sphere_actors.size(); ++j) {
        actors.push_back(score_sphere_actors[j]);
    }
    //    actors.push_back(ring_guides_mesh_actor);
    //    actors.push_back(cornerAnnotation);


}

//------------------------------------------------------------------------------
//std::vector<vtkSmartPointer<vtkProp> > TaskSteadyHand::GetActors() {
//    return actors;
//}

//------------------------------------------------------------------------------
void TaskSteadyHand::SetCurrentToolPosePointer(KDL::Frame &tool_pose,
                                             const int tool_id) {

    tool_current_pose_kdl[tool_id] = &tool_pose;

}

void TaskSteadyHand::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};
//------------------------------------------------------------------------------
void TaskSteadyHand::UpdateActors() {




    // -------------------------------------------------------------------------
    // Find closest points and update frames
    for (int k = 0; k < 1 + (int)bimanual; ++k) {

        // -------------
        // the noise in the pose measurements of the tool made the rendered
        // shadows behave unexpectedly, to fix it we do a 1 step moving
        // averqge of the position of the tool before applying it to the ring

        vtkSmartPointer<vtkMatrix4x4>   tool_pose_filt = vtkSmartPointer<vtkMatrix4x4>::New();
        KDL::Frame tool_current_kdl = *tool_current_pose_kdl[k];

        KDL::Frame tool_current_filt_kdl = tool_current_kdl;
        tool_current_filt_kdl.p = 0.5*(tool_last_pose[k].p + tool_current_kdl.p);

        VTKConversions::KDLFrameToVTKMatrix(tool_current_filt_kdl, tool_pose_filt);
        tool_last_pose[k] = *tool_current_pose_kdl[k];
        // -------------

        // setting the transformations
        tool_current_frame_axes[k]->SetUserMatrix(tool_pose_filt);
        //ring_actor[k]->SetUserMatrix(tool_pose_filt);
        std::vector<double> pose(7, 0.0);
        conversions::KDLFrameToVector(tool_current_filt_kdl, pose);
        double pose_array[7] = {pose[0], pose[1], pose[2], pose[3], pose[4],
            pose[5], pose[6]};
        ring_mesh[0]->SetKinematicPose(pose_array);
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





    //-------------------------------- UPDATE RIGHT GRIPPER
    // map gripper value to an angle
    double grip_posit = (*gripper_position[0]);
    double theta_min=14*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    UpdateGripperLinksPose(tool_last_pose[0], grip_angle, gripper_link_dims,
                           right_gripper_links);

    //-------------------------------- UPDATE LEFT GRIPPER
    KDL::Frame grpr_left_pose = (*tool_current_pose_kdl[1]);
    // map gripper value to an angle
    grip_posit = (*gripper_position[1]);
    grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    UpdateGripperLinksPose(tool_last_pose[1], grip_angle, gripper_link_dims,
                           left_gripper_links);






    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    double positioning_tolerance = 0.003;
    KDL::Vector destination_ring_position;


    // when the active constraints is suddenly enabled the tool makes some
    // initial large movements. To prevent always having a peak at the
    // beginning of the error, data we first activate the active constraint
    // and wait till the error is small before we start the task.
    //
    // First: if the tool is placed close to the starting point while being idle
    // we enable the active constraint.
    if (task_state == SHTaskState::Idle && with_guidance &&
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
    if (task_state == SHTaskState::Idle
        && ( (position_error_norm[0] < 0.001 && ac_parameters.active == 1)
            ||
                (!with_guidance && (ring_center[0] - start_point).Norm() <
                    positioning_tolerance)  ))
    {
        task_state = SHTaskState::ToEndPoint;
        //increment the repetition number
        number_of_repetition ++;
        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
    }

        // If the tool reaches the end point the user needs to go back to
        // the starting point. This counts as a separate repetition of the task
    else if (task_state == SHTaskState::ToEndPoint &&
        (ring_center[0] - end_point).Norm() <
            positioning_tolerance) {
        task_state = SHTaskState::ToStartPoint;
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
    else if (task_state == SHTaskState::ToStartPoint &&
        (ring_center[0] - start_point).Norm() <
            positioning_tolerance) {
        task_state = SHTaskState::RepetitionComplete;
        // calculate and save the score of this repetition
        CalculateAndSaveError();

        ac_parameters.active = 0;
        ac_params_changed = true;
    }

        // User needs to get away from the starting point to switch to idle
        // and in case another repetition is to be performed, the user can
        // flag that by going to the starting position again
    else if (task_state == SHTaskState::RepetitionComplete &&
        (ring_center[0] - idle_point).Norm() <
            positioning_tolerance) {
        task_state = SHTaskState::Idle;
    }

    // update the position of the cone according to the state we're in.
    if (task_state == SHTaskState::Idle) {
        destination_ring_position = start_point;
        destination_ring_actor->GetProperty()->SetColor(SHColors::Green);

    } else if (task_state == SHTaskState::ToStartPoint) {
        destination_ring_position = start_point;
        destination_ring_actor->GetProperty()->SetColor(SHColors::DeepPink);

    } else if (task_state == SHTaskState::ToEndPoint) {
        destination_ring_position = end_point;
        destination_ring_actor->GetProperty()->SetColor(SHColors::DeepPink);
    } else if (task_state == SHTaskState::RepetitionComplete) {
        destination_ring_position = idle_point;
        destination_ring_actor->GetProperty()->SetColor(SHColors::Green);

    }

    // show the destination to the user
    double dt = sin(2 * M_PI * double(destination_ring_counter) / 70);
    destination_ring_counter++;
    destination_ring_actor->SetScale(0.006 + 0.001*dt);

    destination_ring_actor->SetPosition(destination_ring_position[0],
                                        destination_ring_position[1],
                                        destination_ring_position[2]);
    // -------------------------------------------------------------------------
    // Performance Metrics
    UpdateTubeColor();

    // Populate the task state message
    task_state_msg.task_name = "BuzzWire";
    task_state_msg.task_state = (uint8_t)task_state;
    task_state_msg.number_of_repetition = number_of_repetition;
    if (task_state == SHTaskState::ToStartPoint
        || task_state == SHTaskState::ToEndPoint) {

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

    //--------------------------------
    // step the world
    StepDynamicsWorld();

}





//------------------------------------------------------------------------------
void TaskSteadyHand::CalculatedDesiredToolPose() {
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
bool TaskSteadyHand::IsACParamChanged() {
    return ac_params_changed;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters TaskSteadyHand::GetACParameters() {

    ac_params_changed = false;
    // assuming once we read it we can consider it unchanged
    return ac_parameters;
}


//------------------------------------------------------------------------------
void TaskSteadyHand::UpdateTubeColor() {

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
    if(task_state== SHTaskState::ToEndPoint
        || task_state== SHTaskState::ToStartPoint)
        tube_mesh->GetActor()->GetProperty()->SetColor(0.9,
                                                 0.5- 0.4*(error_ratio-0.3),
                                                 0.1);

}

custom_msgs::TaskState TaskSteadyHand::GetTaskStateMsg() {
    return task_state_msg;
}

void TaskSteadyHand::ResetTask() {
    ROS_INFO("Resetting the task.");
    number_of_repetition = 0;
    task_state = SHTaskState::RepetitionComplete;
    ResetOnGoingEvaluation();
    ResetScoreHistory();
}

void TaskSteadyHand::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
    if(task_state== SHTaskState::ToEndPoint ||
        task_state == SHTaskState::ToStartPoint){

        ResetOnGoingEvaluation();
        if(number_of_repetition>0)
            number_of_repetition--;
        task_state = SHTaskState::RepetitionComplete;

    }
}


void TaskSteadyHand::FindAndPublishDesiredToolPose() {

    //----------------------------------------------
    // setting  up haptics

    //std::string master_names[n_arms];
    std::string check_topic_name;

    //getting the name of the arms
    std::stringstream param_name;
    param_name << std::string("/") << slave_names[0] << "/tool_pose_desired";

    ros::Publisher pub_desired[2];
    ros::Publisher pub_wrench_abs[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
                             (param_name.str(), 10);
    ROS_INFO("Will publish on %s", param_name.str().c_str());

    // make sure the masters are in wrench absolute orientation
   // assuming MTMR is always used
    std::string master_topic = "/dvrk/MTMR/set_wrench_body_orientation_absolute";
    pub_wrench_abs[0] = node->advertise<std_msgs::Bool>(master_topic.c_str(), 1);
    std_msgs::Bool wrench_body_orientation_absolute;
    wrench_body_orientation_absolute.data = 1;
    pub_wrench_abs[0].publish(wrench_body_orientation_absolute);
    ROS_INFO("Setting wrench_body_orientation_absolute on %s", master_topic.c_str());

    if(bimanual) {
        //getting the name of the arms
        param_name.str("");
        param_name << std::string("/") << slave_names[1] << "/tool_pose_desired";
        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
                                 (param_name.str(), 10);
        ROS_INFO("Will publish on %s", param_name.str().c_str());

        // make sure the masters are in wrench absolute orientation
        // assuming MTMR is always used
        master_topic = "/dvrk/MTML/set_wrench_body_orientation_absolute";
        pub_wrench_abs[1] = node->advertise<std_msgs::Bool>(master_topic.c_str(), 1);
        pub_wrench_abs[1].publish(wrench_body_orientation_absolute);
        ROS_INFO("Setting wrench_body_orientation_absolute on %s", master_topic.c_str());
    }


    ros::Rate loop_rate(haptic_loop_rate);
    ROS_INFO("The desired pose will be updated at '%f'",
             haptic_loop_rate);

    //---------------------------------------------
    // loop

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
            KDL::Frame tool_desired_pose_in_slave_frame;
            tool_desired_pose_in_slave_frame =
                slave_frame_to_world_frame_tr->Inverse()*
            tool_desired_pose_kdl[n_arm];

            tf::poseKDLToMsg(tool_desired_pose_in_slave_frame, pose_msg.pose);
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

void TaskSteadyHand::CalculateAndSaveError() {

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

double * TaskSteadyHand::GetScoreColor(const double score) {

    //decide the color
    if(score > 90){
        return SHColors::Green;
    }
    else if (score > 80)
        return SHColors::Gold;
    else if(score > 60)
        return SHColors::Orange;
    else
        return SHColors::Red;

}

void TaskSteadyHand::ResetOnGoingEvaluation() {
    posit_error_sum = 0.0;
    posit_error_max = 0.0;
    orient_error_sum = 0.0;
    sample_count = 0;
}

void TaskSteadyHand::ResetScoreHistory() {
    score_history.clear();
    score_history_colors.clear();

    // reset colors to gray
    for (int i = 0; i < n_score_history; ++i) {
        score_sphere_actors[i]->GetProperty()->SetColor(SHColors::Gray);

    }

}




void TaskSteadyHand::InitBullet() {

    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;


    dynamics_world = new btDiscreteDynamicsWorld(dispatcher,
                                                 overlappingPairCache, solver,
                                                 collisionConfiguration);

    dynamics_world->setGravity(btVector3(0, 0, -10));


    btContactSolverInfo& info = dynamics_world->getSolverInfo();
    //optionally set the m_splitImpulsePenetrationThreshold (only used when m_splitImpulse  is enabled)
    //only enable split impulse position correction when the penetration is
    // deeper than this m_splitImpulsePenetrationThreshold, otherwise use the
    // regular velocity/position constraint coupling (Baumgarte).
    info.m_splitImpulsePenetrationThreshold = -0.02f;
    info.m_numIterations = 15;
    info.m_solverMode = SOLVER_USE_2_FRICTION_DIRECTIONS;

}


void TaskSteadyHand::StepDynamicsWorld() {
    ///-----stepsimulation_start-----
    double time_step = (ros::Time::now() - time_last).toSec();

    // simulation seems more realistic when time_step is halved right now!
    dynamics_world->stepSimulation(btScalar(time_step), 60, 1/240.f);
    time_last = ros::Time::now();
}


void TaskSteadyHand::UpdateGripperLinksPose(const KDL::Frame pose,
                                        const double grip_angle,
                                        const std::vector<std::vector<double> > gripper_link_dims,
                                        BulletVTKObject *link_objects[]
) {
    KDL::Frame grpr_links_pose[5];

    //-------------------------------- LINK 0
    grpr_links_pose[0] = pose;
    grpr_links_pose[0].p  = grpr_links_pose[0] * KDL::Vector( 0.0 , 0.0,
                                                              -gripper_link_dims[0][2]/2);
    double x, y, z, w;
    pose.M.GetQuaternion(x,y,z,w);
    double link0_pose[7] = {grpr_links_pose[0].p.x(),
        grpr_links_pose[0].p.y(), grpr_links_pose[0].p.z(),x,y,z,w};
    link_objects[0]->SetKinematicPose(link0_pose);

    //-------------------------------- LINK 1
    grpr_links_pose[1] = pose;
    grpr_links_pose[1].M.DoRotX(-grip_angle);
    grpr_links_pose[1].p =  grpr_links_pose[1] *
        KDL::Vector( 0.0, 0.0, gripper_link_dims[1][2]/2);
    grpr_links_pose[1].M.GetQuaternion(x, y, z, w);

    double link2_pose[7] = {grpr_links_pose[1].p.x(),
        grpr_links_pose[1].p.y(), grpr_links_pose[1].p.z(), x, y, z, w};

    link_objects[1]->SetKinematicPose(link2_pose);

    //-------------------------------- LINK 2
    grpr_links_pose[2] = pose;
    grpr_links_pose[2].M.DoRotX(grip_angle);
    grpr_links_pose[2].p =  grpr_links_pose[2] *
        KDL::Vector( 0.0, 0.0, gripper_link_dims[2][2]/2);
    grpr_links_pose[2].M.GetQuaternion(x, y, z, w);

    double link3_pose[7] = {grpr_links_pose[2].p.x(),
        grpr_links_pose[2].p.y(), grpr_links_pose[2].p.z(), x, y, z, w};

    link_objects[2]->SetKinematicPose(link3_pose);


    //-------------------------------- LINKS 3 and 4
    for (int i = 3; i < 5; ++i) {
        // first find the end point of links 1 and 2 and then add half length
        // of links 3 and 4
        grpr_links_pose[i] = pose;
        grpr_links_pose[i].p =
            grpr_links_pose[i-2] *
                KDL::Vector(0., 0.,gripper_link_dims[i-2][2]/2)
                + grpr_links_pose[i].M *
                    KDL::Vector(0., 0.,gripper_link_dims[i][2]/2);

        grpr_links_pose[i].M.GetQuaternion(x,y,z,w);
        double link_pose[7] = {grpr_links_pose[i].p.x(),
            grpr_links_pose[i].p.y(), grpr_links_pose[i].p.z(),x, y, z, w};

        link_objects[i]->SetKinematicPose(link_pose);
    }

    //------------------------------ ARM

    KDL::Vector shift;
    for (int i = 0; i < 1 + (int)bimanual; ++i) {

        shift=pose.p-rcm[i];

        double norm;
        norm=shift.Norm();

        KDL::Frame orientation;
        orientation.M.UnitX({1,0,0});
        orientation.M.UnitY(shift);
        orientation.M.UnitZ(orientation.M.UnitX()*orientation.M.UnitY());
        orientation.M.UnitX(orientation.M.UnitX()*orientation.M.UnitY());
        orientation.M.GetQuaternion(x,y,z,w);

        orientation.p= (rcm[i]-fabs(rcm[i].Norm()-norm)*shift/(shift.Norm()));

        double arm_pose[7]={orientation.p.x(),
            orientation.p.y(),
            orientation.p.z(),
            x,y,z,w};

        arm[i]->SetKinematicPose(arm_pose);
    }


}

TaskSteadyHand::~TaskSteadyHand() {
    delete [] slave_names;

    ROS_INFO("Destructing Bullet task: %d",
             dynamics_world->getNumCollisionObjects());
    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamics_world->removeCollisionObject(obj);
        delete obj;
    }

}
