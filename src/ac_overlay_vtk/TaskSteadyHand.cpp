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
    mesh_files_dir(stl_file_dir),
    slave_frame_to_world_frame_tr(slave_to_world_tr),
//        show_ref_frames(show_ref_frames),
//        bimanual(biman),
//        with_guidance(with_guidance),
    destination_ring_counter(0),
    ac_params_changed(true),
    task_state(SHTaskState::Idle),
    n_score_history(10),
    time_last(ros::Time::now()) {

    InitBullet();

    slave_names = new std::string[bimanual + 1];

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

    if (bimanual) {
        tool_current_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
        tool_desired_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
        tool_current_pose[1] = vtkSmartPointer<vtkMatrix4x4>::New();
    }

    destination_ring_actor = vtkSmartPointer<vtkActor>::New();

    cellLocator = vtkSmartPointer<vtkCellLocator>::New();

    line1_source = vtkSmartPointer<vtkLineSource>::New();

    line2_source = vtkSmartPointer<vtkLineSource>::New();

    line1_actor = vtkSmartPointer<vtkActor>::New();

    line2_actor = vtkSmartPointer<vtkActor>::New();



    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    double dummy_pose[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    BulletVTKObject *floor = new BulletVTKObject(
        ObjectShape::STATICPLANE,
        ObjectType::DYNAMIC,
        floor_dims, dummy_pose, 0.0, 0,
        0,
        NULL
    );
    dynamics_world->addRigidBody(floor->GetBody());

    BulletVTKObject *board;
    // -------------------------------------------------------------------------
    // Create a cube for the board
    {
        double friction = 0.005;
        double board_dimensions[3] = {0.14, 0.12, 0.01};

        double pose[7]{
            board_dimensions[0] / 2.45, board_dimensions[1] / 2.78,
            -board_dimensions[2] / 2, 0, 0, 0, 1
        };

        std::vector<double> dim = {
            board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, dim,
            pose, 0.0, 0, friction,
            NULL
        );
        board->GetActor()->GetProperty()->SetColor(0.5, 0.5, 0.6);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
    }

    // -------------------------------------------------------------------------
    // Destination ring
    ring_radius = 0.00475;
    double ring_cross_section_radius = 0.0005;
    double source_scales = 0.006;

    vtkSmartPointer<vtkParametricTorus> parametricObject =
        vtkSmartPointer<vtkParametricTorus>::New();
    parametricObject->SetCrossSectionRadius(
        ring_cross_section_radius / source_scales
    );
    parametricObject->SetRingRadius(ring_radius / source_scales);

    vtkSmartPointer<vtkParametricFunctionSource> parametricFunctionSource =
        vtkSmartPointer<vtkParametricFunctionSource>::New();
    parametricFunctionSource->SetParametricFunction(parametricObject);
    parametricFunctionSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> destination_ring_mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    destination_ring_mapper->SetInputConnection(
        parametricFunctionSource->GetOutputPort());
    destination_ring_actor->SetMapper(destination_ring_mapper);
    destination_ring_actor->SetScale(0.004);
    destination_ring_actor->RotateX(90);
    destination_ring_actor->RotateY(-45);
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

    for (int k = 0; k < 1 + (int) bimanual; ++k) {
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
    KDL::Vector base_position = KDL::Vector(0.11, 0.06, 0.025);
    idle_point = base_position + KDL::Vector(-0.056, -0.034, 0.004);

    // -------------------------------------------------------------------------
    // Stand MESH hq

    std::stringstream input_file_dir;
    std::string mesh_file_dir_str;
    input_file_dir.str("");
    input_file_dir << mesh_files_dir
                   << std::string("task_steady_hand_stand.obj");
    mesh_file_dir_str = input_file_dir.str();

    // Define the rotation of the tube mesh
    KDL::Rotation tube_rot(
        KDL::Rotation::RotX(M_PI / 2) *
            KDL::Rotation::RotY(-M_PI / 2));
    KDL::Frame tube_pose;
    tube_pose.M = tube_rot * KDL::Rotation::RotX(-M_PI / 2);
    tube_pose.p = base_position;

    // The stand object orientation follows the orientation of the tube
    KDL::Rotation stand_rot(
        KDL::Rotation::RotZ(45. * M_PI / 180.) *
            KDL::Rotation::RotX(M_PI) *
            tube_rot
    );

    double qx, qy, qz, qw;
    stand_rot.GetQuaternion(qx, qy, qz, qw);
    double stand_pose[7] = {
        base_position[0], base_position[1], base_position[2], qx, qy, qz, qw
    };

    std::vector<double> _dim;
    double friction = 0.001;

    stand_mesh = new
        BulletVTKObject(
        ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
        stand_pose, 0.0, 0, friction, &mesh_file_dir_str
    );

    dynamics_world->addRigidBody(stand_mesh->GetBody());
    actors.push_back(stand_mesh->GetActor());
    stand_mesh->GetActor()->GetProperty()->SetColor(SHColors::Gray);
    //stand_mesh->GetActor()->GetProperty()->SetSpecular(0.8);
    //stand_mesh->GetActor()->GetProperty()->SetSpecularPower(80);
    //tube_mesh->GetActor()->GetProperty()->SetOpacity(0.1);

    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for generating
    // active constraints

    //tube_rot.GetQuaternion(qx, qy,qz, qw);
    //
    //double pose_tube[7]{base_position[0], base_position[1], base_position[2],
    //    qx, qy,qz, qw};
    //
    //_dim = {0.002};
    //input_file_dir.str("");
    //input_file_dir << mesh_files_dir
    //               << std::string("steady_hand_mesh_whole.obj");
    //mesh_file_dir_str = input_file_dir.str();
    //
    //friction = 0.001;
    //
    //tube_mesh = new
    //    BulletVTKObject(ObjectShape::MESH, ObjectType::NOPHYSICS, _dim,
    //                    pose_tube, 0.0, 0, friction,
    //                    &mesh_file_dir_str);
    //
    //dynamics_world->addRigidBody(tube_mesh->GetBody());
    //actors.push_back(tube_mesh->GetActor());
    //tube_mesh->GetActor()->GetProperty()->SetColor(SHColors::Orange);
    //tube_mesh->GetActor()->GetProperty()->SetSpecular(0.8);
    //tube_mesh->GetActor()->GetProperty()->SetSpecularPower(80);
    //tube_mesh->GetActor()->GetProperty()->SetOpacity(0.1);


    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for generating
    // active constraints
    tube_rot.GetQuaternion(qx, qy, qz, qw);

    double pose_tube[7]{
        base_position[0], base_position[1], base_position[2], qx, qy, qz, qw
    };

    _dim = {0.002};
    for (int m = 0; m <3; ++m) {

        input_file_dir.str("");
        input_file_dir << mesh_files_dir
                       << std::string("steady_hand_mesh_part") << m+1 << ".obj";
        mesh_file_dir_str = input_file_dir.str();

        friction = 0.001;

        tube_meshes[m] = new
            BulletVTKObject(
            ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
            pose_tube, 0.0, 0, friction,
            &mesh_file_dir_str
        );

        dynamics_world->addRigidBody(tube_meshes[m]->GetBody());
        actors.push_back(tube_meshes[m]->GetActor());
        tube_meshes[m]->GetActor()->GetProperty()->SetColor(SHColors::Orange);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecular(0.8);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecularPower(80);
        //tube_mesh->GetActor()->GetProperty()->SetOpacity(0.1);
    }
    // -------------------------------------------------------------------------
    // MESH lq
    input_file_dir.str("");
    input_file_dir << mesh_files_dir
                   << std::string("steady_hand_mesh_3_thin.obj");
    mesh_file_dir_str = input_file_dir.str();

    tube_mesh_thin = new
        BulletVTKObject(ObjectShape::MESH, ObjectType::NOPHYSICS, _dim,
                        pose_tube, 0.0, 0, friction,
                        &mesh_file_dir_str);

    actors.push_back(tube_mesh_thin->GetActor());

    //// TODO: Locally transform the mesh so that in the findDesiredPose we
    /// don't repeat the transform every time.

    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(tube_mesh_thin->GetActor()->GetMapper()->GetInput());
    cellLocator->BuildLocator();


    // -------------------------------------------------------------------------
    // Lines
    vtkSmartPointer<vtkPolyDataMapper> line1_mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    line1_actor->SetMapper(line1_mapper);
    line1_actor->GetProperty()->SetLineWidth(3);
    //line1_actor->GetProperty()->SetOpacity(0.8);

    vtkSmartPointer<vtkPolyDataMapper> line2_mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    line2_mapper->SetInputConnection(line2_source->GetOutputPort());
    line2_actor->SetMapper(line2_mapper);
    line2_actor->GetProperty()->SetLineWidth(3);
    //line2_actor->GetProperty()->SetOpacity(0.8);

    // -------------------------------------------------------------------------
    // Closing cylinder

    KDL::Vector distance(0.013116, -0.097892, 0);
    distance = tube_pose*distance;
    KDL::Frame cyl_pose;
    cyl_pose.M = KDL::Rotation::RotZ(M_PI/4)*tube_pose.M;
    cyl_pose.p = distance;
    dir = cyl_pose.M.UnitY();
    cyl_pose.M.GetQuaternion(qx, qy, qz, qw);

    double pose_cyl[7]{cyl_pose.p.x(), cyl_pose.p.y(), cyl_pose.p.z(),
        qx, qy, qz, qw};

    _dim = {0.007, 0.001};
    friction = 0.005;

    supporting_cylinder = new
        BulletVTKObject(ObjectShape::CYLINDER, ObjectType::DYNAMIC, _dim,
                        pose_cyl, 0.0, 0, friction);

    dynamics_world->addRigidBody(supporting_cylinder->GetBody());
    actors.push_back(supporting_cylinder->GetActor());
    supporting_cylinder->GetActor()->GetProperty()->SetColor(SHColors::Orange);
    supporting_cylinder->GetActor()->GetProperty()->SetSpecular(0.7);

    // -------------------------------------------------------------------------
    // Create ring mesh

    _dim = {0.002};

    friction = 50;
    double density = 50000; // kg/m3

    double step = 0.004;

    for (int l = 0; l < ring_num; ++l) {

        double pose[7]{cyl_pose.p.x() + (l+1) * step * dir.x(),
            cyl_pose.p.y() + (l+1) * step * dir.y(),
            cyl_pose.p.z() + (l+1) * step * dir.z(),
            qx, qy, qz, qw};

        input_file_dir.str("");
        input_file_dir << stl_file_dir << std::string("task_steady_hand_ring2.obj");
        mesh_file_dir_str = input_file_dir.str();

        ring_mesh[ring_num - l -1] = new
            BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
                            pose, density, 0, friction,
                            &mesh_file_dir_str);

        dynamics_world->addRigidBody(ring_mesh[ring_num - l -1]->GetBody());
        actors.push_back(ring_mesh[ring_num - l -1]->GetActor());
        ring_mesh[ring_num - l -1]->GetActor()->GetProperty()->SetColor(SHColors::Turquoise);
        ring_mesh[ring_num - l -1]->GetActor()->GetProperty()->SetSpecular(0.7);

        ring_mesh[ring_num - l -1]->GetBody()->setContactStiffnessAndDamping(2000, 100);
    }

    start_point  = cyl_pose.p + ring_num * step *dir;
    end_point = {0.015, 0.004, 0};
    KDL::Frame T;
    T.M = stand_rot * KDL::Rotation::RotX(M_PI/2);
    T.p = base_position;
    end_point  = T * end_point;

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
        double gripper_pose[7]{0, 0, 0, 0, 0, 0, 1};
        gripper_link_dims =
            {{0.003, 0.003, 0.003}
             , {0.002, 0.002, 0.007}
             , {0.002, 0.002, 0.007}};

        grippers[0] = new ThreeLinkGripper(gripper_link_dims);
        grippers[1] = new ThreeLinkGripper(gripper_link_dims);

        for (int j = 0; j < 2; ++j) {
            grippers[j]->AddToWorld(dynamics_world);
            grippers[j]->AddToActorsVector(actors);
        }


        // Cylinders resembling the robotic arms

        KDL::Vector cam_position(0.118884, 0.27565, 0.14583);

        rcm[0] = {cam_position.x() - 0.1, cam_position.y(), cam_position.z()
            + 0.05};
        rcm[1] = {cam_position.x() + 0.1, cam_position.y(), cam_position.z()
            + 0.05};

        for (int i = 0; i < 1 + (int)bimanual; ++i) {

            std::vector<double> arm_dim = { 0.002, rcm[i].Norm()*2};

            arm[i] = new BulletVTKObject(
                ObjectShape::CYLINDER,
                ObjectType::KINEMATIC, arm_dim, gripper_pose, 0.0
            );
            dynamics_world->addRigidBody(arm[i]->GetBody());
            actors.push_back(arm[i]->GetActor());
            arm[i]->GetActor()->GetProperty()->SetColor(1, 1, 1);
            arm[i]->GetActor()->GetProperty()->SetSpecularPower(50);
            arm[i]->GetActor()->GetProperty()->SetSpecular(0.8);
            //arm[i]->GetActor()->GetProperty()->SetOpacity(1.0);

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

    //actors.push_back(stand_mesh_actor);
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


    ring_pose = ring_mesh[ring_in_action]->GetPose();

    // check if any of the grippers have grasped the ring in action
    for (int i = 0; i < 2; ++i) {
        gripper_in_contact[i] = grippers[i]->IsGraspingObject(dynamics_world,
                                          ring_mesh[ring_in_action]->GetBody());
    }

    // change the color of the grasped ring
    if (gripper_in_contact[0] || gripper_in_contact[1])
        ring_mesh[ring_in_action]->GetActor()->GetProperty()
                ->SetColor(1.,0.,0.);
    else
        ring_mesh[ring_in_action]->GetActor()->GetProperty()
                ->SetColor(SHColors::Green);

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
    double theta_min=0*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    grippers[0]->SetPoseAndJawAngle(tool_last_pose[0], grip_angle);

    //-------------------------------- UPDATE LEFT GRIPPER
    KDL::Frame grpr_left_pose = (*tool_current_pose_kdl[1]);
    // map gripper value to an angle
    grip_posit = (*gripper_position[1]);
    grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    grippers[1]->SetPoseAndJawAngle(tool_last_pose[1], grip_angle);


    UpdateGripperLinksPose(tool_last_pose[0], 0);
    UpdateGripperLinksPose(tool_last_pose[1], 1);

    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    double positioning_tolerance = 0.006;
    KDL::Vector destination_ring_position;

    // first run
    if (task_state == SHTaskState::Idle) {
       destination_ring_position = start_point;
    } else
        destination_ring_position = end_point;


    // when the active constraints is suddenly enabled the tool makes some
    // initial large movements. To prevent always having a peak at the
    // beginning of the error, data we first activate the active constraint
    // and wait till the error is small before we start the task.
    //
    // First: if the tool is placed close to the starting point while being idle
    // we enable the active constraint.
    if (task_state == SHTaskState::Idle && with_guidance &&
        (ring_pose.p - start_point).Norm() <
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
        && ( (position_error_norm< 0.001 && ac_parameters.active == 1)
            ||
                (!with_guidance && (ring_pose.p - start_point).x()>0.0 )  ))
    {
        std::cout << " transition to start" << std::endl;
        task_state = SHTaskState::ToEndPoint;
        destination_ring_actor->RotateY(90);
        //increment the repetition number
        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
        destination_ring_actor->GetProperty()->SetColor(SHColors::DeepPink);
    }

    //    // If the tool reaches the end point the user needs to go back to
    //    // the starting point. This counts as a separate repetition of the task
    //else if (task_state == SHTaskState::ToEndPoint &&
    //    (ring_pose.p - end_point).Norm() <
    //        positioning_tolerance) {
    //    task_state = SHTaskState::ToStartPoint;
    //    destination_ring_actor->RotateY(90);
    //
    //    //increment the repetition number
    //    number_of_repetition++;

        //// calculate and save the score of this repetition
        //CalculateAndSaveError();
        //
        //// save starting time
        //start_time = ros::Time::now();
        //// reset score related vars
        //ResetOnGoingEvaluation();

    //}
    //    // If the tool reaches the start point while in ToStartPoint state,
    //    // we can mark the task complete
    //else if (task_state == SHTaskState::ToStartPoint &&
    //    (ring_pose.p - start_point).Norm() <
    //        positioning_tolerance) {
    //    task_state = SHTaskState::RepetitionComplete;
    //    destination_ring_actor->RotateY(-90);
    //
    //    // calculate and save the score of this repetition
    //    CalculateAndSaveError();
    //
    //    ac_parameters.active = 0;
    //    ac_params_changed = true;
    //}

        // User needs to get away from the starting point to switch to idle
        // and in case another repetition is to be performed, the user can
        // flag that by going to the starting position again
    //else if (task_state == SHTaskState::RepetitionComplete &&
    //    (ring_pose.p - idle_point).Norm() <
    //        positioning_tolerance)
    else if (task_state == SHTaskState::ToEndPoint &&
            (ring_pose.p - end_point).Norm() <
                positioning_tolerance)
    {
        std::cout << " transition to finish" << std::endl;

        task_state = SHTaskState::Idle;
        destination_ring_actor->RotateY(-90);
        //Reset tube color
        for (int i = 0; i < 3; ++i) {
            tube_meshes[i]->GetActor()->GetProperty()->SetColor
                (SHColors::Orange);
        }
        supporting_cylinder->GetActor()->GetProperty()->SetColor(SHColors::Orange);
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
            (SHColors::Turquoise);

        ring_in_action +=  1;

        // ---------- ADDED --------------
        // calculate and save the score of this repetition
        CalculateAndSaveError();

        ac_parameters.active = 0;
        ac_params_changed = true;

        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
        //------------------------------

        destination_ring_actor->GetProperty()->SetColor(SHColors::Green);
    }

    // update the position of the ring according to the state we're in.
    //if (task_state == SHTaskState::Idle) {
    //
    //
    ////} else if (task_state == SHTaskState::ToStartPoint) {
    ////    destination_ring_position = start_point;
    ////    destination_ring_actor->GetProperty()->SetColor(SHColors::DeepPink);
    //
    //} else if (task_state == SHTaskState::ToEndPoint) {
    //
    //
    ////} else if (task_state == SHTaskState::RepetitionComplete) {
    ////    destination_ring_position = idle_point;
    ////    destination_ring_actor->GetProperty()->SetColor(SHColors::Green);
    //
    //}

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
    task_state_msg.number_of_repetition = ring_in_action+1;
    if (task_state == SHTaskState::ToEndPoint){
        //|| task_state == SHTaskState::ToStartPoint) {

        task_state_msg.time_stamp = (ros::Time::now() - start_time).toSec();
        task_state_msg.error_field_1 = position_error_norm;
        task_state_msg.error_field_1 = orientation_error_norm;
//        if(bimanual)
//            task_state_msg.error_field_2 = position_error_norm[1];

        // calculate score to show to user
        if (bimanual) {
            posit_error_sum +=
                0.5 * (position_error_norm);
            orient_error_sum +=
                0.5 *(orientation_error_norm);
        }
        else {
            posit_error_sum += position_error_norm;
            orient_error_sum += orientation_error_norm;
        }

        if(posit_error_max < position_error_norm)
            posit_error_max = position_error_norm;

        sample_count++;

    } else {
        task_state_msg.time_stamp = 0.0;
        task_state_msg.error_field_1 = 0.0;
        task_state_msg.error_field_2 = 0.0;
    }


    //if(bimanual) {
    //    // change connection lines colors according to ring1 to ring2's distance
    //    //double rings_distance = (ring_center[1] - ring_center[0]).Norm();
    //    double ideal_distance = 0.007;
    //    double error_ratio = 3 * fabs(rings_distance - ideal_distance)
    //        / ideal_distance;
    //    if (error_ratio > 1.0)
    //        error_ratio = 1.0;
    //    line1_actor->GetProperty()->SetColor(0.9, 0.9 - 0.7 * error_ratio,
    //                                         0.9 - 0.7 * error_ratio);
    //    line2_actor->GetProperty()->SetColor(0.9, 0.9 - 0.7 * error_ratio,
    //                                         0.9 - 0.7 * error_ratio);
    //    line1_source->Update();
    //    line2_source->Update();
    //}

    //--------------------------------
    // step the world
    StepDynamicsWorld();

}





//------------------------------------------------------------------------------
void TaskSteadyHand::CalculatedDesiredToolPose(const KDL::Frame ring_pose,
                                               const KDL::Frame tool_pose,
                                               KDL::Frame &desired_tool_pose) {
    // NOTE: All the closest points are on the wire mesh

    //---------------------------------------------------------------------
    // Find the desired orientation
    // We use two vectors to estimate the tangent of the direction of the
    // tube. Assuming that the ref frame of the ring is on the center of its
    // circle and and the circle lie on the x-y plane.
    // Imagine two points on the inner circle of the ring such that one
    // point is put on the intersection of the x axis of the ring with its
    // inner circle and the other point is simillar but along the y axis.
    // The estimated tangent is the cross product of the two vectors
    // connecting these points to their corresponding closest point on the
    // surface of the thin tube (after normalization). This is just a quick
    // and non-ideal approximation.
    // Note that we could have used the central point instead of the tool
    // point but that vector gets pretty small and unstable when we're
    // close to the desired pose.

    // Another important thing to note here is that we want to apply the
    // guidance force to the tool at the end (and not the ring). so we find
    // the transformation that would take the current ring pose to the
    // desired one and add that to the current pose of the tool.


    // Note that to find the closest point to the mesh, we need to take the
    // points to the local reference farame of the mesh object
    // So we need the its pose and inv
    KDL::Frame mesh_pose_inv = tube_mesh_thin->GetPose().Inverse();

    // ----------------------- FIRST CLOSEST POINT
    //Find the closest cell to the the central point
    KDL::Vector center_point_in_mesh_local = mesh_pose_inv*ring_pose.p;

    double ring_central_point_in_mesh_local[3] = {center_point_in_mesh_local[0],
        center_point_in_mesh_local[1],
        center_point_in_mesh_local[2]};

    double closest_point[3] = {0.0, 0.0, 0.0};
    double closestPointDist2; //the squared distance to the closest point
    vtkIdType cell_id; //the cell id of the cell containing the closest point
    int subId;

    cellLocator->Update();
    cellLocator->FindClosestPoint(ring_central_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);
    // take the closest point to the current pose of the object
    KDL::Vector closest_point_to_center_point = tube_mesh_thin->GetPose() *
        KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    // ----------------------- SECOND CLOSEST POINT
    //Find the closest cell to the grip point
    KDL::Vector radial_x_point_kdl = ring_pose *
        KDL::Vector(ring_radius, 0.0, 0.0);
    KDL::Vector radial_x_point_in_mesh_local_kdl =mesh_pose_inv*radial_x_point_kdl;

    double radial_x_point_in_mesh_local[3] = {radial_x_point_in_mesh_local_kdl[0],
        radial_x_point_in_mesh_local_kdl[1],
        radial_x_point_in_mesh_local_kdl[2]};

    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_x_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);
    // take the closest point to the current pose of the object
    KDL::Vector closest_point_to_x_point = tube_mesh_thin->GetPose() *
        KDL::Vector(closest_point[0],closest_point[1],closest_point[2]);

    // ----------------------- THIRD CLOSEST POINT
    //Find the closest cell to the radial tool point
    KDL::Vector radial_y_point_kdl = ring_pose *
        KDL::Vector(0., 0., ring_radius);
    KDL::Vector radial_y_point_in_mesh_local_kdl = mesh_pose_inv*radial_y_point_kdl;

    double radial_y_point_in_mesh_local[3] = {radial_y_point_in_mesh_local_kdl[0],
        radial_y_point_in_mesh_local_kdl[1],
        radial_y_point_in_mesh_local_kdl[2]};

    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_y_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);
    // take the closest point to the current pose of the object
    KDL::Vector closest_point_to_y_point = tube_mesh_thin->GetPose() *
        KDL::Vector(closest_point[0],closest_point[1],closest_point[2]);


    // Find the vector from ring center to the corresponding closest point on
    // the thin mesh
    KDL::Vector ring_center_to_cp = closest_point_to_center_point - ring_pose.p;

    KDL::Frame ring_tranform_to_its_desired_pose;
    // desired pose only when the ring is close to the wire.if it is too
    // far we don't want fixtures
    if (ring_center_to_cp.Norm() < 5 * ring_radius) {

        // Desired position is one that puts the center of the ring on the
        // center of the mesh.

        //---------------------------------------------------------------------
        // Find the desired position of the ring assuming that the length
        // of the thin tube is negligible
        ring_tranform_to_its_desired_pose.p = ring_center_to_cp;

        KDL::Vector desired_z, desired_y, desired_x;

        KDL::Vector point_y_to_cp =
                closest_point_to_y_point - radial_y_point_kdl;

        KDL::Vector point_x_to_cp =
                closest_point_to_x_point - radial_x_point_kdl;

        desired_z = point_x_to_cp / point_x_to_cp.Norm();
        desired_x = -point_y_to_cp / point_y_to_cp.Norm();
        desired_y = desired_z * desired_x;

        // make sure axes are perpendicular and normal
        desired_y = desired_y / desired_y.Norm();
        desired_x = desired_y * desired_z;
        desired_x = desired_x / desired_x.Norm();
        desired_z = desired_x * desired_y;
        desired_z = desired_z / desired_z.Norm();

        ring_tranform_to_its_desired_pose.M =
                KDL::Rotation(desired_x, desired_y, desired_z)
                * ring_pose.M.Inverse();

        // we add the displacement that would take the ring to it's desired
        // pose to the current pose of the tool;
        desired_tool_pose.p =
                ring_tranform_to_its_desired_pose.p + tool_pose.p;
        desired_tool_pose.M =
                ring_tranform_to_its_desired_pose.M * tool_pose.M;

        //------------------------------------------------------------------
        // Calculate errors
        position_error_norm = ring_tranform_to_its_desired_pose.p.Norm();
        KDL::Vector rpy;
        ring_tranform_to_its_desired_pose.M.GetRPY(rpy[0],
                                                     rpy[1],
                                                     rpy[2]);
        orientation_error_norm = rpy.Norm();

    } else {
        desired_tool_pose = tool_pose;
        // due to the delay in teleop loop this will create some wrneches if
        // the guidance is still active
    }


    // draw the connection lines for debug
    line1_source->SetPoint1(ring_pose.p[0],
                            ring_pose.p[1],
                            ring_pose.p[2]);
    line1_source->SetPoint2(closest_point_to_center_point[0],
                            closest_point_to_center_point[1],
                            closest_point_to_center_point[2]);
    line2_source->SetPoint1(radial_y_point_kdl[0],
                            radial_y_point_kdl[1],
                            radial_y_point_kdl[2]);
    line2_source->SetPoint2(closest_point_to_y_point[0],
                            closest_point_to_y_point[1],
                            closest_point_to_y_point[2]);


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
    double error_ratio = ( (orientation_error_norm / max_orient_error)
        + 2* (position_error_norm / max_pos_error)) /3;

    if (error_ratio > 1.3)
        error_ratio = 1.3;
    else if(error_ratio < 0.3)
        error_ratio = 0.3;

//    score_sphere_actors->GetProperty()->SetColor(error_ratio, 1 - error_ratio,
//                                                0.1);
    if(task_state== SHTaskState::ToEndPoint){
        //|| task_state== SHTaskState::ToStartPoint){
        for (int i = 0; i < 3; ++i) {
            tube_meshes[i]->GetActor()->GetProperty()->SetColor(0.9,
                                                                0.5- 0.4*(error_ratio-0.3),
                                                                0.1);
        }
        supporting_cylinder->GetActor()->GetProperty()->SetColor(0.9,
                                                                 0.5- 0.4*(error_ratio-0.3),
                                                                 0.1);
    }

}

custom_msgs::TaskState TaskSteadyHand::GetTaskStateMsg() {
    return task_state_msg;
}

void TaskSteadyHand::ResetTask() {
    ROS_INFO("Resetting the task.");
    ring_in_action = 0;
    //task_state = SHTaskState::RepetitionComplete
    // -----------ADDED-----------------------
    task_state = SHTaskState::Idle;
    // ---------------------------------------
    ResetOnGoingEvaluation();
    ResetScoreHistory();
}

void TaskSteadyHand::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
    if(task_state== SHTaskState::ToEndPoint){
    //|| task_state == SHTaskState::ToStartPoint){
        ResetOnGoingEvaluation();
        if(ring_in_action>0)
            ring_in_action--;
        //task_state = SHTaskState::RepetitionComplete;
        // -----------ADDED-----------------------
        task_state = SHTaskState::Idle;
        // ---------------------------------------
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
        double ring_position[3];
        ring_mesh[0]->GetActor()->GetPosition(ring_position);
        //ring_center[0] = KDL::Vector(ring_position[0], ring_position[1],
        //                             ring_position[2]);
        //
        //if(bimanual){
        //    VTKConversions::KDLFrameToVTKMatrix(*tool_current_pose_kdl[1],
        //                                        tool_current_pose[1]);
        //    ring_center[1] = *tool_current_pose_kdl[1] *
        //        KDL::Vector(0.0, ring_radius, 0.0);
        //}


        CalculatedDesiredToolPose(ring_pose,  *tool_current_pose_kdl[0],
                                  tool_desired_pose_kdl[0]);

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
                                            int gripper_side) {

    //------------------------------ ARM

    int i = gripper_side;
    double x,y,z,w;
    KDL::Vector shift;

    shift = pose.p-rcm[i];

    double norm;
    norm = shift.Norm();

    if (norm > rcm[i].Norm())
    {
        double displacement=norm-rcm[i].Norm();
        displacement = displacement - 4*gripper_link_dims[2][2]/2;
        KDL::Frame orientation;
        orientation.M.UnitX({1,0,0});
        orientation.M.UnitY(shift/shift.Norm());
        orientation.M.UnitZ(orientation.M.UnitX()*orientation.M.UnitY());
        orientation.M.UnitX(orientation.M.UnitY()*orientation.M.UnitZ());
        orientation.M.GetQuaternion(x,y,z,w);

        orientation.p= (rcm[i]+displacement*(shift)/shift.Norm());
        double arm_pose[7]={orientation.p.x(),
                orientation.p.y(),
                orientation.p.z(),
                x,y,z,w};
        arm[i]->SetKinematicPose(arm_pose);

    } else
    {
        double displacement=-norm+rcm[i].Norm();
        displacement = displacement + 4*gripper_link_dims[2][2]/2;
        KDL::Frame orientation;
        orientation.M.UnitX({1,0,0});
        orientation.M.UnitY(shift/shift.Norm());
        orientation.M.UnitZ(orientation.M.UnitX()*orientation.M.UnitY());
        orientation.M.UnitX(orientation.M.UnitY()*orientation.M.UnitZ());
        orientation.M.GetQuaternion(x,y,z,w);
        orientation.p= (rcm[i]-displacement*(shift)/shift.Norm());
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
