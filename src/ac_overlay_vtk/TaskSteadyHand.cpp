//
// Created by nima on 4/18/17.
//

#include <custom_conversions/Conversions.h>
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include "TaskSteadyHand.h"


TaskSteadyHand::TaskSteadyHand(
    const std::string mesh_file_dir,
    const bool show_ref_frames,
    const bool biman,
    const bool with_guidance,
    const double haptic_loop_rate,
    const std::string slave_names_in[],
    KDL::Frame *slave_to_world_tr)
    :
    VTKTask(show_ref_frames, biman, with_guidance, haptic_loop_rate),
    mesh_files_dir(mesh_file_dir),
    slave_frame_to_world_frame_tr(slave_to_world_tr),
    destination_ring_counter(0),
    ac_params_changed(true),
    task_state(SHTaskState::Idle),
    n_score_history(10),
    time_last(ros::Time::now())
{

    InitBullet();

    slave_names = new std::string[bimanual + 1];

    *slave_names = *slave_names_in;

    // -------------------------------------------------------------------------
    //  ACTIVE CONSTRAINT
    // -------------------------------------------------------------------------
    // these parameters could be set as ros parameters too but since
    // they change during the task I am hard coding them here.
    ac_parameters[0].method = 0; // 0 for visco/elastic
    ac_parameters[0].active = 0;

    ac_parameters[0].max_force = 4.0;
    ac_parameters[0].linear_elastic_coeff = 1000.0;
    ac_parameters[0].linear_damping_coeff = 10.0;

    //ac_parameters[0].max_torque = 0.03;
    ac_parameters[0].max_torque = 0.0;
    ac_parameters[0].angular_elastic_coeff = 0.04;
    ac_parameters[0].angular_damping_coeff = 0.002;

    // prevent tools from hitting things at the initializiation
    tool_current_pose[0].p = KDL::Vector(0.1, 0.1, 0.1);
    tool_current_pose[1].p = KDL::Vector(0.1, 0.2, 0.1);
    // -------------------------------------------------------------------------
    //  INITIALIZING GRAPHICS ACTORS
    // -------------------------------------------------------------------------

    tool_current_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();

    //if (bimanual) {
    //    tool_current_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
    //    tool_desired_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
    //}

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
        double board_dimensions[3] = {0.18, 0.132, 0.008};

        double pose[7]{
            board_dimensions[0] / 3, board_dimensions[1] / 2,
            -board_dimensions[2] / 2, 0, 0, 0, 1
        };

        std::vector<double> dim = {
            board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new BulletVTKObject(
            ObjectShape::BOX, ObjectType::DYNAMIC, dim,
            pose, 0.0, 0, friction);
        board->GetActor()->GetProperty()->SetColor(colors.Gray);

        dynamics_world->addRigidBody(board->GetBody());
        actors.push_back(board->GetActor());
    }

    // -------------------------------------------------------------------------
    // Destination ring
    ring_radius = 0.005;
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
    destination_ring_actor->SetScale(0.002);
    destination_ring_actor->RotateX(90);
    destination_ring_actor->RotateY(-60);
    destination_ring_actor->GetProperty()->SetColor(colors.OrangeRed);
    //destination_ring_actor->GetProperty()->SetOpacity(0.5);

    // -------------------------------------------------------------------------
    // FRAMES
    //vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
    //    vtkSmartPointer<vtkAxesActor>::New();
    //
    //task_coordinate_axes->SetXAxisLabelText("");
    //task_coordinate_axes->SetYAxisLabelText("");
    //task_coordinate_axes->SetZAxisLabelText("");
    //task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    //task_coordinate_axes->SetShaftType(vtkAxesActor::LINE_SHAFT);
    //task_coordinate_axes->SetTipType(vtkAxesActor::SPHERE_TIP);

    //for (int k = 0; k < 1 + (int) bimanual; ++k) {
    for (int k = 0; k < 1 ; ++k) {
        tool_current_frame_axes[k]->SetXAxisLabelText("");
        tool_current_frame_axes[k]->SetYAxisLabelText("");
        tool_current_frame_axes[k]->SetZAxisLabelText("");
        tool_current_frame_axes[k]->SetTotalLength(0.007, 0.007, 0.007);
        tool_current_frame_axes[k]->SetShaftType(vtkAxesActor::LINE_SHAFT);
        tool_current_frame_axes[k]->SetTipType(vtkAxesActor::SPHERE_TIP);

        tool_desired_frame_axes[k]->SetXAxisLabelText("");
        tool_desired_frame_axes[k]->SetYAxisLabelText("");
        tool_desired_frame_axes[k]->SetZAxisLabelText("");
        tool_desired_frame_axes[k]->SetTotalLength(0.007, 0.007, 0.007);
        tool_desired_frame_axes[k]->SetShaftType(vtkAxesActor::LINE_SHAFT);
        tool_desired_frame_axes[k]->SetTipType(vtkAxesActor::SPHERE_TIP);
    }


    // -------------------------------------------------------------------------
    // Add all actors to a vector
    if (show_ref_frames) {
        //actors.push_back(task_coordinate_axes);
        //
        //for (int k = 0; k < 1 + (int)bimanual; ++k) {
        for (int k = 0; k < 1 ; ++k) {
            actors.push_back(tool_current_frame_axes[k]);
            actors.push_back(tool_desired_frame_axes[k]);
        }
    }


    // hard coding the position of of the destinations
    // if the base is rotated the destinations will not be valid anymore...
    KDL::Vector base_position = KDL::Vector(0.11-0.016, 0.08, 0.025+0.035);
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
        KDL::Rotation::RotZ(-1.0*M_PI /18. ) *
        KDL::Rotation::RotX(M_PI / 2) *
            KDL::Rotation::RotY(-M_PI / 2));
    KDL::Frame tube_pose;
    tube_pose.M = tube_rot * KDL::Rotation::RotX(-M_PI / 2);
    tube_pose.p = base_position;

    // The stand object orientation follows the orientation of the tube
    KDL::Rotation stand_rot(
        KDL::Rotation::RotZ(40. * M_PI / 180.) *
            KDL::Rotation::RotX(M_PI) *
            tube_rot);

    double qx, qy, qz, qw;
    stand_rot.GetQuaternion(qx, qy, qz, qw);
    double stand_pose[7] = {
        base_position[0], base_position[1], base_position[2], qx, qy,
        qz, qw
    };

    std::vector<double> _dim;
    double friction = 0.001;
    stand_mesh = new
        BulletVTKObject(
        ObjectShape::MESH, ObjectType::DYNAMIC, _dim,
        stand_pose, 0.0, 0, friction, &mesh_file_dir_str);

    dynamics_world->addRigidBody(stand_mesh->GetBody());
    actors.push_back(stand_mesh->GetActor());
    stand_mesh->GetActor()->GetProperty()->SetColor(colors.GrayLight);
    //stand_mesh->GetActor()->GetProperty()->SetSpecular(0.8);
    //stand_mesh->GetActor()->GetProperty()->SetSpecularPower(80);
    //tube_mesh->GetActor()->GetProperty()->SetOpacity(0.1);

    // -------------------------------------------------------------------------
    // Stand CUBic MESH

    std::vector<double> dim_cube={0.025+0.01,0.025+0.01,(0.11-0.016-0.025)/2};

    double pose_cube[7] = {
        base_position[0]+0.02*cos(M_PI/4), base_position[1]-0.02*cos
            (M_PI/4), (0.11-0.016-0.025)
            /4, qx,
        qy,
        qz, qw
    };

    stand_cube = new
        BulletVTKObject(
        ObjectShape::BOX, ObjectType::DYNAMIC, dim_cube,
        pose_cube, 0.0, 0, friction, &mesh_file_dir_str);

    dynamics_world->addRigidBody(stand_cube->GetBody());
    actors.push_back(stand_cube->GetActor());
    stand_cube->GetActor()->GetProperty()->SetColor(colors.GrayLight);

    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for generating
    // active constraints
    tube_rot.GetQuaternion(qx, qy, qz, qw);

    double pose_tube[7]{
        base_position[0], base_position[1], base_position[2], qx, qy, qz,
        qw};

    for (int m = 0; m <4; ++m) {

        input_file_dir.str("");
        input_file_dir << mesh_files_dir
                       << std::string("tube_quarter_mesh_bis") << m+1 <<".obj";
        mesh_file_dir_str = input_file_dir.str();

        tube_meshes[m] = new
            BulletVTKObject(
            ObjectShape::MESH, ObjectType::DYNAMIC, {},
            pose_tube, 0.0, 0, friction, &mesh_file_dir_str);

        dynamics_world->addRigidBody(tube_meshes[m]->GetBody());
        actors.push_back(tube_meshes[m]->GetActor());
        tube_meshes[m]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecular(1);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecularPower(100);
        //tube_meshes[m]->GetActor()->GetProperty()->SetSpecularColor(colors
        //                                                                 .White);
//        tube_meshes[m]->GetActor()->GetProperty()->SetOpacity(0.1);
    }
    // -------------------------------------------------------------------------
    // MESH thin
    input_file_dir.str("");
    input_file_dir << mesh_files_dir
                   << std::string("tube_whole_thin.obj");
    mesh_file_dir_str = input_file_dir.str();
    tube_mesh_thin = new
        BulletVTKObject(ObjectShape::MESH, ObjectType::NOPHYSICS, _dim,
                        pose_tube, 0.0, 0, friction, &mesh_file_dir_str);
    actors.push_back(tube_mesh_thin->GetActor());

    //// TODO: Locally transform the mesh so that in the findDesiredPose we
    /// don't repeat the transform every time.

    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(tube_mesh_thin->GetActor()->GetMapper()->GetInput());
    cellLocator->BuildLocator();


    // -------------------------------------------------------------------------
    // Closing cylinder
    KDL::Frame cyl_pose;
    cyl_pose.M = KDL::Rotation::RotZ(39./180.*M_PI)*tube_pose.M;
    cyl_pose.p = tube_pose*KDL::Vector(0.009, -0.093, -0.033); // ADDED Z
    dir = cyl_pose.M.UnitY();
    cyl_pose.M.GetQuaternion(qx, qy, qz, qw);

    //double pose_cyl[7]{cyl_pose.p.x(), cyl_pose.p.y(), cyl_pose.p.z(),
    //    qx, qy, qz, qw};

    //_dim = {0.006, 0.002};
    //
    //closing_cylinder = new
    //    BulletVTKObject(ObjectShape::CYLINDER, ObjectType::DYNAMIC, _dim,
    //                    pose_cyl, 0.0);
    //
    //dynamics_world->addRigidBody(closing_cylinder->GetBody());
    //actors.push_back(closing_cylinder->GetActor());
    //closing_cylinder->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
    //closing_cylinder->GetActor()->GetProperty()->SetSpecular(1);
    //closing_cylinder->GetActor()->GetProperty()->SetSpecularPower(100);
    //closing_cylinder->GetActor()->GetProperty()->SetSpecularColor(colors.White);

    // -------------------------------------------------------------------------
    //// Lines
    //vtkSmartPointer<vtkPolyDataMapper> line1_mapper =
    //    vtkSmartPointer<vtkPolyDataMapper>::New();
    //line1_mapper->SetInputConnection(line1_source->GetOutputPort());
    //line1_actor->SetMapper(line1_mapper);
    //line1_actor->GetProperty()->SetLineWidth(3);
    //line1_actor->GetProperty()->SetColor(colors.Coral);
    //
    ////line1_actor->GetProperty()->SetOpacity(0.8);
    //
    //vtkSmartPointer<vtkPolyDataMapper> line2_mapper =
    //    vtkSmartPointer<vtkPolyDataMapper>::New();
    //line2_mapper->SetInputConnection(line2_source->GetOutputPort());
    //line2_actor->SetMapper(line2_mapper);
    //line2_actor->GetProperty()->SetLineWidth(3);
    ////line2_actor->GetProperty()->SetOpacity(0.8);

    // -------------------------------------------------------------------------
    // Create ring meshes
    friction = 50;
    double density = 50000; // kg/m3
    double step = 0.0045;

    KDL::Rotation rings_orient = cyl_pose.M * KDL::Rotation::RotX(M_PI/2);
    rings_orient.GetQuaternion(qx, qy, qz, qw);

    for (int l = 0; l < ring_num; ++l) {

        double pose[7]{cyl_pose.p.x() + (l+1) * step * dir.x(),
            cyl_pose.p.y() + (l+1) * step * dir.y(),
            cyl_pose.p.z() + (l+1) * step * dir.z(),
            qx, qy, qz, qw};

        input_file_dir.str("");
        //input_file_dir <<mesh_file_dir << std::string("torus_D10mm_d1.2mm.obj");
        input_file_dir <<mesh_file_dir << std::string("torus_D10mm_d1.2mm"
                                                          ".obj");

        mesh_file_dir_str = input_file_dir.str();

        ring_mesh[ring_num - l -1] = new
            BulletVTKObject(ObjectShape::MESH, ObjectType::DYNAMIC, {},
                            pose, density, 0, friction,
                            &mesh_file_dir_str);

        dynamics_world->addRigidBody(ring_mesh[ring_num - l -1]->GetBody());
        actors.push_back(ring_mesh[ring_num - l -1]->GetActor());
        ring_mesh[ring_num-l-1]->GetActor()->GetProperty()->SetColor(colors.Turquoise);
        ring_mesh[ring_num-l-1]->GetBody()->setContactStiffnessAndDamping
            (5, 1);
        ring_mesh[ring_num-l-1]->GetBody()->setRollingFriction(btScalar(0.01));
        ring_mesh[ring_num-l-1]->GetBody()->setSpinningFriction(btScalar(0.01));




        //// --------- separation cylinders --------------------

            _dim = {0.0004, 0.005};

            KDL::Rotation cyl_orient = cyl_pose.M *
                KDL::Rotation::RotX(M_PI / 2);
            cyl_orient.GetQuaternion(qx, qy, qz, qw);

            double pose_cyl[7]{
                cyl_pose.p.x() + (l -0.5+1) * step * dir.x(), cyl_pose.p.y()
                    + (l -0.5+ 0.2) * step * dir.y(), cyl_pose.p.z()
                    + (l-0.5) * step * dir.z(), qx, qy, qz, qw
            };

            sep_cylinder[l] = new
                BulletVTKObject(
                ObjectShape::CYLINDER, ObjectType::KINEMATIC, _dim,
                pose_cyl, 0.0
            );

            dynamics_world->addRigidBody(sep_cylinder[l]->GetBody());
            actors.push_back(sep_cylinder[l]->GetActor());
            sep_cylinder[l]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);


        ////---------------------------------------------------

    }

    start_point  = cyl_pose.p + (ring_num + 3) * step *dir;
    end_point = {0.015, -0.0022, 0};
    KDL::Frame T (stand_rot * KDL::Rotation::RotX(M_PI/2), base_position);
    end_point  = T * end_point;

    // -------------------------------------------------------------------------
    // Create Forceps
    {
        KDL::Frame forceps_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
        forceps_pose.M.DoRotZ(M_PI/2);
        forceps[0] = new Forceps(mesh_file_dir, forceps_pose);
        forceps_pose.p.x(0.07);
        forceps[1] = new Forceps(mesh_file_dir, forceps_pose);

        for (int j = 0; j < 2; ++j) {
            forceps[j]->AddToWorld(dynamics_world);
            forceps[j]->AddToActorsVector(actors);
        }
    }

    // -------------------------------------------------------------------------
    // Create tool rods
    {
        // Cylinders resembling the PSM rods
        KDL::Vector cam_posit(0.118884, 0.27565, 0.14583);

        rcm[0] = {cam_posit.x()-0.1, cam_posit.y(), cam_posit.z()+ 0.05};
        rcm[1] = {cam_posit.x()+0.1, cam_posit.y(), cam_posit.z()+ 0.05};

        double gripper_pose[7]{0.04, 0.2, 0.1, 0, 0, 0, 1};
        for (int i = 0; i < 1 + (int)bimanual; ++i) {
            std::vector<double> arm_dim = { 0.002, rcm[i].Norm()*2};
            arm[i] = new BulletVTKObject(
                ObjectShape::CYLINDER,
                ObjectType::KINEMATIC, arm_dim, gripper_pose, 0.0);
            dynamics_world->addRigidBody(arm[i]->GetBody());
            actors.push_back(arm[i]->GetActor());
            arm[i]->GetActor()->GetProperty()->SetColor(colors.GrayDark);
            //arm[i]->GetActor()->GetProperty()->SetOpacity(0.1);
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
        actor->GetProperty()->SetColor(colors.Gray);
        actor->SetPosition(0.09- (double)i * 0.006, 0.12 + (double)i * 0.0001,
                           0.005);
        score_sphere_actors.push_back(actor);
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
    tool_current_pose_ptr[tool_id] = &tool_pose;
}

void TaskSteadyHand::SetCurrentGripperpositionPointer(double &grip_position, const int
tool_id) {
    gripper_position[tool_id] = &grip_position;
};
//------------------------------------------------------------------------------
void TaskSteadyHand::UpdateActors() {


    // check if any of the forceps have grasped the ring in action
    for (int i = 0; i < 2; ++i) {
        gripper_in_contact[i] = forceps[i]->IsGraspingObject(dynamics_world,
                                                             ring_mesh[ring_in_action]->GetBody());
    }


    // we need to interpolate the pose of the ring for the high-freq haptic
    // loop. Otherwise the haptics wiill go unstable.
    ring_pose = ring_mesh[ring_in_action]->GetPose();
    if(gripper_in_contact[0])
        tool_to_ring_tr[0] = tool_current_pose[0].Inverse() * ring_pose;
    if(gripper_in_contact[1])
        tool_to_ring_tr[1] = tool_current_pose[1].Inverse() * ring_pose ;

    //// change the color of the grasped ring
    //if (gripper_in_contact[0] || gripper_in_contact[1])
    //    ring_mesh[ring_in_action]->GetActor()->GetProperty()
    //            ->SetColor(1.,0.,0.);
    //else
    //    ring_mesh[ring_in_action]->GetActor()->GetProperty()
    //            ->SetColor(Colors::Green);

    // -------------------------------------------------------------------------
    // Find closest points and update frames
//    for (int k = 0; k < 1 + (int)bimanual; ++k) {
//
//        // -------------
//        // the noise in the pose measurements of the tool made the rendered
//        // shadows behave unexpectedly, to fix it we do a 1 step moving
//        // averqge of the position of the tool before applying it to the ring
//
//        vtkSmartPointer<vtkMatrix4x4>   tool_pose_filt = vtkSmartPointer<vtkMatrix4x4>::New();
//
//        KDL::Frame tool_current_filt_kdl = tool_current_pose[k];
//        tool_current_filt_kdl.p = 0.5*(tool_last_pose[k].p + tool_current_pose[k].p);
//
//        VTKConversions::KDLFrameToVTKMatrix(tool_current_filt_kdl, tool_pose_filt);
//        tool_current_frame_axes[k]->SetUserMatrix(tool_pose_filt);
//
//        tool_last_pose[k] = tool_current_pose[k];
//        // -------------
//    }


    UpdateCurrentAndDesiredReferenceFrames(tool_current_pose,
                                           tool_desired_pose);

    //-------------------------------- UPDATE RIGHT GRIPPER
    // map gripper value to an angle
    double grip_posit = (*gripper_position[0]);
    double theta_min=0*M_PI/180;
    double theta_max=20*M_PI/180;
    double grip_angle = theta_max*(grip_posit)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    //KDL::Frame temp_pose;
    //double dx = sin(2 * M_PI * double(destination_ring_counter) / 50);
    //temp_pose.p = KDL::Vector(0.09 + dx*0.02, 0.15, 0.09);
    //temp_pose.M.DoRotY(M_PI);
    //temp_pose.M.DoRotZ(M_PI/2);

    forceps[0]->SetPoseAndJawAngle(tool_current_pose[0], grip_angle);

    //-------------------------------- UPDATE LEFT GRIPPER
    // map gripper value to an angle
    grip_posit = (*gripper_position[1]);
    grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    forceps[1]->SetPoseAndJawAngle(tool_current_pose[1], grip_angle);

    UpdateToolRodsPose(tool_current_pose[0], 0);
    UpdateToolRodsPose(tool_current_pose[1], 1);

    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    KDL::Vector destination_ring_position;

    // first run
    if (task_state == SHTaskState::Idle) {
        destination_ring_position = start_point;
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
            (colors.Orange);
    } else
        destination_ring_position = end_point;


    // when the active constraints is suddenly enabled the tool makes some
    // initial large movements. To prevent always having a peak at the
    // beginning of the error, data we first activate the active constraint
    // and wait till the error is small before we start the task.
    //
    //// First: if the tool is placed close to the starting point while being idle
    //// we enable the active constraint.
    //if (task_state == SHTaskState::Idle && with_guidance &&
    //    (ring_pose.p - start_point).Norm() <
    //        positioning_tolerance) {
    //    // Make sure the active constraint is inactive
    //    if (ac_parameters.active == 0) {
    //        ac_parameters.active = 1;
    //        ac_params_changed = true;
    //    }
    //}
    double positioning_tolerance = 0.006;

    // if we finished the task in the last run, switch to idle now
    if (task_state == SHTaskState::Finished)
        task_state = SHTaskState::Idle;

    // if we are idle and the ring in action is clode to the start point, and
    // in the positive x side of it then start the acquisition
    if (task_state == SHTaskState::Idle
        && ((ring_pose.p - start_point).x() > 0.0
            && ((ring_pose.p - start_point).Norm() < positioning_tolerance) ))
    {
        std::cout << " Transition to start" << std::endl;
        task_state = SHTaskState::OnGoing;
        destination_ring_actor->RotateY(100);
        //increment the repetition number
        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
            (colors.Orange);
    }

    else if (task_state == SHTaskState::OnGoing &&
        (ring_pose.p - end_point).Norm() <
            positioning_tolerance)
    {
        std::cout << " Transition to finish" << std::endl;

        task_state = SHTaskState::Finished;
        destination_ring_actor->RotateY(-100);
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
            (colors.Turquoise);

        ring_in_action +=  1;

        KDL::Vector base_position = KDL::Vector(0.11-0.016, 0.08, 0.025+0.035);

        double pose_cube[7] = {
            base_position[0]+0.02*cos(M_PI/4), base_position[1]-0.02*cos
                (M_PI/4), (0.11-0.016-0.025)
                /4,0,0,0,1};
        sep_cylinder[ring_num-ring_in_action]->SetKinematicPose(pose_cube);

        //dynamics_world->removeRigidBody(sep_cylinder[ring_in_action]->GetBody());

        // calculate and save the score of this repetition
        CalculateAndSaveError();


        // save starting time
        start_time = ros::Time::now();

        // reset score related vars
        ResetOnGoingEvaluation();

    }

    // active constraint activation
    if(task_state == SHTaskState::OnGoing){
        for (int i = 0; i < 2; ++i) {
            if(with_guidance && gripper_in_contact[i]
                && !ac_parameters[i] .active){
                ac_parameters[i].active = 1;
                ac_params_changed = true;
            }
            else if(!gripper_in_contact[i] && ac_parameters[i].active){
                ac_parameters[i].active = 0;
                ac_params_changed = true;
            }

        }
    }

    // show the destination to the user
    double dt = sin(2 * M_PI * double(destination_ring_counter) / 70);
    destination_ring_counter++;
    destination_ring_actor->SetScale(0.002 + 0.0004*dt);

    destination_ring_actor->SetPosition(destination_ring_position[0],
                                        destination_ring_position[1],
                                        destination_ring_position[2]);

    // -------------------------------------------------------------------------
    // Performance Metrics
    UpdateTubeColor();

    // Populate the task state message
    task_state_msg.task_name = "SteadyHand";
    task_state_msg.task_state = (uint8_t)task_state;
    task_state_msg.number_of_repetition = uchar(ring_in_action+1);
    task_state_msg.uint_slot = (uchar)gripper_in_contact[0] +
        (uchar)gripper_in_contact[1] *(uchar)2;

    if (task_state == SHTaskState::OnGoing) {

        task_state_msg.time_stamp = (ros::Time::now() - start_time).toSec();
        task_state_msg.error_field_1 = position_error_norm;
        task_state_msg.error_field_1 = orientation_error_norm;
        //      if(bimanual)
        //          task_state_msg.error_field_2 = position_error_norm[1];

        // calculate score to show to user
        posit_error_sum += position_error_norm;
        orient_error_sum += orientation_error_norm;

        if(posit_error_max < position_error_norm)
            posit_error_max = position_error_norm;

        sample_count++;

    } else {
        task_state_msg.time_stamp = 0.0;
        task_state_msg.error_field_1 = 0.0;
        task_state_msg.error_field_2 = 0.0;
    }

    //--------------------------------
    // step the world
    StepDynamicsWorld();

}





//------------------------------------------------------------------------------
void TaskSteadyHand::CalculatedDesiredToolPose(const KDL::Frame ring_pose,
                                               KDL::Frame &transform_to_desired_ring_pose) {
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
        KDL::Vector(0., ring_radius, 0.f);
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

    // desired pose only when the ring is close to the wire.if it is too
    // far we don't want fixtures
    if (ring_center_to_cp.Norm() < 5 * ring_radius) {

        // Desired position is one that puts the center of the ring on the
        // center of the mesh.

        //---------------------------------------------------------------------
        // Find the desired position of the ring assuming that the length
        // of the thin tube is negligible
        transform_to_desired_ring_pose.p = ring_center_to_cp;

        KDL::Vector desired_z, desired_y, desired_x;

        KDL::Vector point_y_to_cp =
            closest_point_to_y_point - radial_y_point_kdl;

        KDL::Vector point_x_to_cp =
            closest_point_to_x_point - radial_x_point_kdl;

        desired_x = -point_x_to_cp / point_x_to_cp.Norm();
        desired_y = -point_y_to_cp / point_y_to_cp.Norm();
        desired_z = desired_x * desired_y;

        // make sure axes are perpendicular and normal
        desired_z = desired_z / desired_z.Norm();
        desired_x = desired_y * desired_z;
        desired_x = desired_x / desired_x.Norm();
        desired_y = desired_z * desired_x;
        desired_y = desired_y / desired_y.Norm();

        transform_to_desired_ring_pose.M =
            KDL::Rotation(desired_x, desired_y, desired_z)
                * ring_pose.M.Inverse();



        //------------------------------------------------------------------
        // Calculate errors
        position_error_norm = transform_to_desired_ring_pose.p.Norm();
        KDL::Vector rpy;
        transform_to_desired_ring_pose.M.GetRPY(rpy[0],
                                                rpy[1],
                                                rpy[2]);
        orientation_error_norm = rpy.Norm();

        // draw the connection lines for debug
        line1_source->SetPoint1(radial_x_point_kdl[0],
                                radial_x_point_kdl[1],
                                radial_x_point_kdl[2]);
        line1_source->SetPoint2(closest_point_to_x_point[0],
                                closest_point_to_x_point[1],
                                closest_point_to_x_point[2]);
        line2_source->SetPoint1(radial_y_point_kdl[0],
                                radial_y_point_kdl[1],
                                radial_y_point_kdl[2]);
        line2_source->SetPoint2(closest_point_to_y_point[0],
                                closest_point_to_y_point[1],
                                closest_point_to_y_point[2]);

    } else {
        transform_to_desired_ring_pose = KDL::Frame();
        // due to the delay in teleop loop this will create some wrneches if
        // the guidance is still active
    }




}


//------------------------------------------------------------------------------
bool TaskSteadyHand::IsACParamChanged() {
    return ac_params_changed;
}


//------------------------------------------------------------------------------
custom_msgs::ActiveConstraintParameters* TaskSteadyHand::GetACParameters() {

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
    //double error_ratio = ( (orientation_error_norm / max_orient_error)
    //    + 2* (position_error_norm / max_pos_error)) /3;
    //double error_ratio = (orientation_error_norm / max_orient_error);

    double error_ratio = (position_error_norm / max_pos_error);

    if (error_ratio > 1.3)
        error_ratio = 1.3;
    else if(error_ratio < 0.3)
        error_ratio = 0.3;

//    score_sphere_actors->GetProperty()->SetColor(error_ratio, 1 - error_ratio,
//                                                0.1);
    if(task_state== SHTaskState::OnGoing){
        ring_mesh[ring_in_action]->GetActor()->GetProperty()
                                 ->SetColor(colors.Orange[0],
                                            colors.Orange[1]- 0.6*
                                                (error_ratio-0.3),
                                            colors.Orange[2]);
    }

}

custom_msgs::TaskState TaskSteadyHand::GetTaskStateMsg() {
    return task_state_msg;
}

void TaskSteadyHand::ResetTask() {
    ROS_INFO("Resetting the task.");
    ring_in_action = 0;
    //task_state = SHTaskState::RepetitionComplete
    task_state = SHTaskState::Idle;
    ResetOnGoingEvaluation();
    ResetScoreHistory();
}

void TaskSteadyHand::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");
    if(task_state== SHTaskState::OnGoing){
        //|| task_state == SHTaskState::ToStartPoint){
        ResetOnGoingEvaluation();
        task_state = SHTaskState::Idle;
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

    //if(bimanual) {
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
    //}


    ros::Rate loop_rate(haptic_loop_rate);
    ROS_INFO("The desired pose will be updated at '%f'",
             haptic_loop_rate);

    //---------------------------------------------
    // loop

    while (ros::ok())
    {

        // find the center of the ring
//        double ring_position[3];
//        ring_mesh[0]->GetActor()->GetPosition(ring_position);
        tool_current_pose[0] = *tool_current_pose_ptr[0];
        tool_current_pose[1] = *tool_current_pose_ptr[1];

        KDL::Frame transform_to_desired_ring_pose;
        KDL::Frame interpolated_ring_pose;
        if(gripper_in_contact[0])
            interpolated_ring_pose = tool_current_pose[0] * tool_to_ring_tr[0];
        else if (gripper_in_contact[1])
            interpolated_ring_pose = tool_current_pose[1] * tool_to_ring_tr[1];

        CalculatedDesiredToolPose(interpolated_ring_pose, transform_to_desired_ring_pose);

        // publish desired poses
        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {

            if(gripper_in_contact[n_arm]) {
                // here find trhe desired tool pose if it is in contact with
                // the ring

                // we add the displacement that would take the ring to it's desired
                // pose to the current pose of the tool;
                tool_desired_pose[n_arm].p =
                    transform_to_desired_ring_pose.p +
                        tool_current_pose[n_arm].p;
                tool_desired_pose[n_arm].M =
                    transform_to_desired_ring_pose.M *
                        tool_current_pose[n_arm].M;

            }
            else {
                tool_desired_pose[n_arm]= tool_current_pose[n_arm];
            }

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            KDL::Frame tool_desired_pose_in_slave_frame;
            tool_desired_pose_in_slave_frame =
                slave_frame_to_world_frame_tr->Inverse()*
                    tool_desired_pose[n_arm];

            tf::poseKDLToMsg(tool_desired_pose_in_slave_frame, pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/slave_frame";
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
        return colors.Green;
    }
    else if (score > 80)
        return colors.Gold;
    else if(score > 60)
        return colors.Orange;
    else
        return colors.Red;

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
        score_sphere_actors[i]->GetProperty()->SetColor(colors.Gray);

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
    dynamics_world->stepSimulation(btScalar(time_step), 90, 1/380.f);
    time_last = ros::Time::now();
}


void TaskSteadyHand::UpdateToolRodsPose(
    const KDL::Frame pose,
    int gripper_side
) {

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
        displacement = displacement - 4* 0.002/2;
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
        displacement = displacement + 4* 0.002/2;
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

void TaskSteadyHand::UpdateCurrentAndDesiredReferenceFrames(
    const KDL::Frame current_pose[2],
    const KDL::Frame desired_pose[2]
) {

    vtkSmartPointer<vtkMatrix4x4> tool_desired_pose_vtk[2];
    vtkSmartPointer<vtkMatrix4x4> tool_current_pose_vtk[2];

    //for (int k = 0; k < 1 + (int)bimanual; ++k) {
    for (int k = 0; k < 1 ; ++k) {
        tool_desired_pose_vtk[k] = vtkSmartPointer<vtkMatrix4x4>::New();
        VTKConversions::KDLFrameToVTKMatrix(desired_pose[k],
                                            tool_desired_pose_vtk[k]);
        tool_desired_frame_axes[k]->SetUserMatrix(tool_desired_pose_vtk[k]);

        tool_current_pose_vtk[k] = vtkSmartPointer<vtkMatrix4x4>::New();
        VTKConversions::KDLFrameToVTKMatrix(current_pose[k],
                                            tool_current_pose_vtk[k]);
        tool_current_frame_axes[k]->SetUserMatrix(tool_current_pose_vtk[k]);
    }

}
