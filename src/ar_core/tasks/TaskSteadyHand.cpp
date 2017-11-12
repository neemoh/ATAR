//
// Created by nima on 4/18/17.
//

#include <custom_conversions/Conversions.h>
#include "src/ar_core/VTKConversions.h"
#include <vtkCubeSource.h>
#include <boost/thread/thread.hpp>
#include <vtkParametricTorus.h>
#include "TaskSteadyHand.h"
#include <vtkSphereSource.h>


TaskSteadyHand::TaskSteadyHand(ros::NodeHandlePtr n)
        :
        SimTask(n, 500),
        destination_ring_counter(0),
        ac_params_changed(true),
        task_state(SHTaskState::Idle)
{

    bool ar_mode = false;
    int n_views = 3;
    bool one_window_per_view = false;
    bool borders_off  = true;
    std::vector<int> view_resolution = {640, 480};
    std::vector<int> window_positions={1280, 0};

    graphics = std::make_unique<Rendering>(n, view_resolution, ar_mode, n_views,
                                           one_window_per_view, borders_off,window_positions);

    // Define two tools
    slaves[0] = new Manipulator(nh, "/dvrk/PSM1_DUMMY",
                                "/position_cartesian_current",
                                "/gripper_position_current");

    slaves[1] = new Manipulator(nh, "/dvrk/PSM2_DUMMY",
                                "/position_cartesian_current",
                                "/gripper_position_current");

    // set the manipulators to follow cam pose for correct kinematics
    // calibration. Cam pose here is cam_0.
    graphics->SetManipulatorInterestedInCamPose(slaves[0]);
    graphics->SetManipulatorInterestedInCamPose(slaves[1]);

    // prevent tools from hitting things at the initialization
    tool_current_pose[0].p = KDL::Vector(0.1, 0.1, 0.1);
    tool_current_pose[1].p = KDL::Vector(0.1, 0.2, 0.1);

    // -------------------------------------------------------------------------
    //  INITIALIZING GRAPHICS ACTORS
    tool_current_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes[0] = vtkSmartPointer<vtkAxesActor>::New();
    tool_current_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();
    tool_desired_frame_axes[1] = vtkSmartPointer<vtkAxesActor>::New();

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
    std::vector<double> floor_dims = {0., 0., 1., -0.5};
    SimObject *floor = new SimObject(ObjectShape::STATICPLANE,
                                     ObjectType::DYNAMIC, floor_dims);
    dynamics_world->addRigidBody(floor->GetBody());

    SimObject *board;
    // -------------------------------------------------------------------------
    // Create a cube for the board
    {
        double friction = 0.05;
        double board_dimensions[3] = {0.18, 0.132, 0.008};

        KDL::Frame pose(KDL::Rotation::Quaternion( 0., 0., 0., 1.)
                , KDL::Vector( board_dimensions[0] / 3,
                               board_dimensions[1] / 2,
                               -board_dimensions[2]/ 2) );

        std::vector<double> dim = {
                board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose);
        board->GetActor()->GetProperty()->SetColor(colors.Gray);
        AddSimObjectToTask(board);
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

    bool show_ref_frames = false;
    // -------------------------------------------------------------------------
    // Add all graphics_actors to a vector
    if (show_ref_frames) {
        //graphics_actors.push_back(task_coordinate_axes);
        //
        //for (int k = 0; k < 1 + (int)bimanual; ++k) {
        for (int k = 0; k < 1 ; ++k) {
            graphics_actors.push_back(tool_current_frame_axes[k]);
            graphics_actors.push_back(tool_desired_frame_axes[k]);
        }
    }


    // hard coding the position of of the destinations
    // if the base is rotated the destinations will not be valid anymore...
    KDL::Vector base_position = KDL::Vector(0.11-0.016, 0.08, 0.025+0.035);

    // -------------------------------------------------------------------------
    // Stand MESH hq

    // Define the rotation of the tube mesh

    pose_tube =   KDL::Frame( KDL::Rotation::RotX(M_PI/2)*
                              KDL::Rotation::RotY(-10./180.*M_PI),
                              base_position);
    KDL::Frame stand_frame = pose_tube;
    stand_frame.M.DoRotY(-49./180.*M_PI);

    std::vector<double> _dim;
    double friction = 0.001;
    stand_mesh = new
            SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                      RESOURCES_DIRECTORY+"/mesh/task_steady_hand_stand.obj"
            , stand_frame, 0.0, friction);

    AddSimObjectToTask(stand_mesh);

    stand_mesh->GetActor()->GetProperty()->SetColor(colors.GrayLight);
    //stand_mesh->GetActor()->GetProperty()->SetSpecular(0.8);
    //stand_mesh->GetActor()->GetProperty()->SetSpecularPower(80);
    //tube_mesh->GetActor()->GetProperty()->SetOpacity(0.1);

    // -------------------------------------------------------------------------
    // Stand CUBic MESH

    std::vector<double> dim_cube={0.025+0.01,0.025+0.01,(0.11-0.016-0.025)/2};

    KDL::Frame pose_cube = stand_frame;
    pose_cube.p = KDL::Vector( base_position[0]+0.015*cos(M_PI/4),
                               base_position[1]-0.03*cos
                                       (M_PI/4), (0.11-0.016-0.025)/4) ;

    stand_cube = new
            SimObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim_cube,
                      pose_cube);
    AddSimObjectToTask(stand_cube);
    stand_cube->GetActor()->GetProperty()->SetColor(colors.GrayLight);

    // -------------------------------------------------------------------------
    // MESH hq is for rendering and lq is for generating
    // active constraints
    std::stringstream input_file_dir;
    std::string mesh_file_dir_str;
    for (int m = 0; m <4; ++m) {

        input_file_dir.str("");
        input_file_dir << RESOURCES_DIRECTORY
                       << std::string
                               ("/mesh/task_steady_hand_tube_quarter_mesh")
                       << m+1 <<".obj";
        mesh_file_dir_str = input_file_dir.str();

        tube_meshes[m] = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC
                , mesh_file_dir_str, pose_tube, 0.0, friction);
        tube_meshes[m]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecular(1);
        tube_meshes[m]->GetActor()->GetProperty()->SetSpecularPower(100);
        AddSimObjectToTask(tube_meshes[m]);
    }

    // -------------------------------------------------------------------------
    // MESH thin
    tube_mesh_thin = new
            SimObject(ObjectShape::MESH, ObjectType::NOPHYSICS
            ,RESOURCES_DIRECTORY+"/mesh/task_steady_hand_tube_whole_thin.obj",
                      pose_tube, 0.0 ,friction);
    //graphics_actors.push_back(tube_mesh_thin->GetActor());

    //// TODO: Locally transform the mesh so that in the findDesiredPose we
    /// don't repeat the transform every time.

    // CLOSEST POINT will be found on the low quality mesh
    cellLocator->SetDataSet(tube_mesh_thin->GetActor()->GetMapper()->GetInput());
    cellLocator->BuildLocator();


    // -------------------------------------------------------------------------
    // Closing cylinder
    KDL::Frame ring_holder_bar_pose;
    ring_holder_bar_pose.M = KDL::Rotation::RotZ(87./180.*M_PI)*stand_frame.M;
    ring_holder_bar_pose.p = pose_tube*KDL::Vector(-0.094, -0.034, 0.010 ); //

    dir = ring_holder_bar_pose.M.UnitX();

    // -------------------------------------------------------------------------
    // Create ring meshes
    friction = 50;
    double density = 50000; // kg/m3
    double step = 0.0045;

    KDL::Rotation rings_orient = ring_holder_bar_pose.M *
                                 KDL::Rotation::RotY(M_PI/2);

    for (int l = 0; l < ring_num; ++l) {

        KDL::Frame pose(rings_orient
                , KDL::Vector( ring_holder_bar_pose.p.x() + (l+1) * step * dir.x(),
                               ring_holder_bar_pose.p.y() + (l+1) * step * dir.y(),
                               ring_holder_bar_pose.p.z()  + (l+1) * step * dir.z()) );

        ring_mesh[ring_num - l -1] = new
                SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                          RESOURCES_DIRECTORY+"/mesh/task_steady_hand_torus_D10mm_d1.2mm.obj"
                , pose, density, friction);

        AddSimObjectToTask(ring_mesh[ring_num - l -1]);
        ring_mesh[ring_num-l-1]->GetActor()->GetProperty()->SetColor(colors.Turquoise);
        ring_mesh[ring_num-l-1]->GetBody()->setContactStiffnessAndDamping
                (3000, 100);
        ring_mesh[ring_num-l-1]->GetBody()->setRollingFriction(btScalar(0.01));
        ring_mesh[ring_num-l-1]->GetBody()->setSpinningFriction(btScalar(0.01));


        //// --------- separation cylinders --------------------

        _dim = {0.0004, 0.005};

        KDL::Frame pose_cyl(rings_orient
                , KDL::Vector(ring_holder_bar_pose.p.x() + (l -0.5+1) * step * dir.x(),
                              ring_holder_bar_pose.p.y() + (l -0.5+ 0.2) * step * dir.y(),
                              ring_holder_bar_pose.p.z()+ (l-0.5) * step *
                                                          dir.z()) );
        sep_cylinder[l] = new
                SimObject(ObjectShape::CYLINDER, ObjectType::KINEMATIC, _dim,
                          pose_cyl);
        sep_cylinder[l]->GetActor()->GetProperty()->SetColor(colors.BlueDodger);
        AddSimObjectToTask(sep_cylinder[l]);
    }

    start_point  = ring_holder_bar_pose.p + (ring_num + 3) * step *dir;
    end_point = pose_tube * KDL::Vector(-0.012, 0.0, -0.01);
    // -------------------------------------------------------------------------
    // Create SimForceps
    {
        KDL::Frame forceps_init_pose = KDL::Frame(KDL::Vector(0.05, 0.11, 0.08));
        forceps_init_pose.M.DoRotZ(M_PI/2);
        forceps[0] = new SimForceps(forceps_init_pose);
        forceps_init_pose.p.x(0.07);
        forceps[1] = new SimForceps(forceps_init_pose);

        AddSimMechanismToTask(forceps[0]);
        AddSimMechanismToTask(forceps[1]);
    }

    // -------------------------------------------------------------------------
    // Create tool rods
    {
        // Cylinders resembling the PSM rods
        KDL::Vector cam_posit(0.118884, 0.27565, 0.14583);

        rcm[0] = {cam_posit.x()-0.1, cam_posit.y(), cam_posit.z()+ 0.05};
        rcm[1] = {cam_posit.x()+0.1, cam_posit.y(), cam_posit.z()+ 0.05};

        KDL::Frame gripper_pose(rings_orient , KDL::Vector(.04, 0.2, 0.1) );

        for (int i = 0; i < 2; ++i) {
            std::vector<double> arm_dim = { 0.002, rcm[i].Norm()*2};
            rods[i] = new SimObject(ObjectShape::CYLINDER, ObjectType::KINEMATIC,
                                   arm_dim, gripper_pose);
            rods[i]->GetActor()->GetProperty()->SetColor(colors.GrayDark);
            rods[i]->GetBody()->setContactStiffnessAndDamping(2000, 100);
            AddSimObjectToTask(rods[i]);
        }
    }

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
    // Error history spheres

    vtkSmartPointer<vtkSphereSource>  source =
            vtkSmartPointer<vtkSphereSource>::New();

    source->SetRadius(0.002);
    source->SetPhiResolution(15);
    source->SetThetaResolution(15);
    vtkSmartPointer<vtkPolyDataMapper> sphere_mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    sphere_mapper->SetInputConnection(source->GetOutputPort());

    for (int i = 0; i < ring_num; ++i) {

        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(sphere_mapper);
        actor->GetProperty()->SetColor(colors.Gray);
        actor->SetPosition(0.115 - (double)i * 0.006, 0.12 + (double)i * 0.0001,
                           0.005);
        score_sphere_actors.push_back(actor);
    }

    graphics_actors.emplace_back(line1_actor);
    graphics_actors.emplace_back(line2_actor);
    graphics_actors.emplace_back(destination_ring_actor);
    for (int j = 0; j < score_sphere_actors.size(); ++j) {
        graphics_actors.emplace_back(score_sphere_actors[j]);
    }

    graphics->AddActorsToScene(GetActors());

    // Publisher for the task state
    publisher_task_state = n->advertise<custom_msgs::TaskState>(
            "/atar/task_state", 1);
};
//------------------------------------------------------------------------------
void TaskSteadyHand::TaskLoop() {

    // check if any of the forceps have grasped the ring in action
    for (int i = 0; i < 2; ++i) {
        gripper_in_contact_last[i] = gripper_in_contact[i];
        gripper_in_contact[i] = forceps[i]->IsGraspingObject(dynamics_world,
                                                             ring_mesh[ring_in_action]->GetBody());
    }

    // -------------------------------------------------------------------------
    // NOTE1: We want to activate the haptic guidance for each tool when it
    // grasps the ring. That, however, will cause a sudden high wrench being
    // applied. To prevent this we can interpolate the desired pose the
    // moment the ring is grasped. The duration of this interpolation is
    // ac_soft_start_duration

    // NOTE2: The  desired pose of the ring is calculated from its current
    // pose. The problem is the current pose of the ring comes from the
    // physics simulation which happens much slower than the haptics thread.
    // To prevent this low freq from causing instability, we estimate the
    // pose of the ring by saving its transformation with respect to the
    // tool-tip at the moment of grasp and from that moment on we use that
    // transform with the pose of the tool, that is updated at the haptics
    // thread frq. This is of course based on the assumption that the
    // relative pose of the ring with respect to the tool does not change,
    // but it actually does sometimes. One can update the ring to tool
    // transformation at every physics loop, but since there is a delay
    // between the motion of the ring and the pose of the tool this will only
    // cause more instability. So what I thought of doing at the end is to
    // regularly check the ring/tool drift and if it is more than a threshold
    // update the transformation and also reset the interpolation counter to
    // make the transition smooth.

    ring_pose = ring_mesh[ring_in_action]->GetPose();

    // calculate the ring to estimated ring drift
    double drift = (ring_pose.p - estimated_ring_pose.p).Norm();

    // Update the tool to ring tr if we just grasped the ring or if the drift
    // has grown too large
    if( (gripper_in_contact[0] & !gripper_in_contact_last[0]) || drift > 0.001)
        tool_to_ring_tr[0] = tool_current_pose[0].Inverse() * ring_pose;
    if( (gripper_in_contact[1] & !gripper_in_contact_last[1]) || drift > 0.001)
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
    UpdateCurrentAndDesiredReferenceFrames(tool_current_pose,
                                           tool_desired_pose);

    //-------------------------------- UPDATE RIGHT GRIPPER
    // map gripper value to an angle
    double theta_min=0*M_PI/180;
    double theta_max=20*M_PI/180;
    double grip_angle = theta_max*(gripper_angle[0])/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    forceps[0]->SetPoseAndJawAngle(tool_current_pose[0], grip_angle);

    //-------------------------------- UPDATE LEFT GRIPPER
    // map gripper value to an angle
    grip_angle = theta_max*(gripper_angle[1]+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    forceps[1]->SetPoseAndJawAngle(tool_current_pose[1], grip_angle);

    UpdateToolRodsPose(tool_current_pose[0], 0);
    UpdateToolRodsPose(tool_current_pose[1], 1);

    // -------------------------------------------------------------------------
    // Task logic
    // -------------------------------------------------------------------------
    KDL::Vector destination_ring_position;

    if (task_state == SHTaskState::Idle) {
        destination_ring_position = start_point;
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
                (colors.Orange);
    } else
        destination_ring_position = end_point;


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
        ROS_INFO(" Started new repetition");
        task_state = SHTaskState::OnGoing;
        destination_ring_actor->RotateY(100);
        //increment the repetition number
        // save starting time
        start_time = ros::Time::now();
        // reset score related vars
        ResetOnGoingEvaluation();
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
                (colors.Orange);
        stand_cube->GetActor()->GetProperty()->SetColor(colors.Coral);

    }

    else if (task_state == SHTaskState::OnGoing &&
             (ring_pose.p - end_point).Norm() <
             positioning_tolerance)
    {
        task_state = SHTaskState::Finished;
        destination_ring_actor->RotateY(-100);
        ring_mesh[ring_in_action]->GetActor()->GetProperty()->SetColor
                (colors.Turquoise);
        if(ring_in_action<(ring_num-1))
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
        stand_cube->GetActor()->GetProperty()->SetColor(colors.GrayLight);

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
    UpdateRingColor();

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

        if(orient_error_max < orientation_error_norm)
            orient_error_max = orientation_error_norm;

        sample_count++;

    } else {
        task_state_msg.time_stamp = 0.0;
        task_state_msg.error_field_1 = 0.0;
        task_state_msg.error_field_2 = 0.0;
    }

    publisher_task_state.publish(task_state_msg);

}





//------------------------------------------------------------------------------
void TaskSteadyHand::CalculatedDesiredRingPose(
        const KDL::Frame ring_pose,
        KDL::Frame &desired_ring_pose
) {
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

    //static KDL::Frame tube_mesh_pose = tube_mesh_thin->GetPose();
    static KDL::Frame tube_mesh_pose = pose_tube;
    static KDL::Frame tube_mesh_pose_inv = tube_mesh_pose.Inverse();

    // ----------------------- FIRST CLOSEST POINT
    //Find the closest cell to the the central point
    KDL::Vector center_point_in_mesh_local = tube_mesh_pose_inv*ring_pose.p;

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
    KDL::Vector closest_point_to_center_point = tube_mesh_pose *
                                                KDL::Vector(closest_point[0], closest_point[1], closest_point[2]);

    // ----------------------- SECOND CLOSEST POINT
    //Find the closest cell to the grip point
    KDL::Vector radial_x_point_kdl = ring_pose *
                                     KDL::Vector(ring_radius, 0.0, 0.0);
    KDL::Vector radial_x_point_in_mesh_local_kdl =tube_mesh_pose_inv*radial_x_point_kdl;

    double radial_x_point_in_mesh_local[3] = {radial_x_point_in_mesh_local_kdl[0],
                                              radial_x_point_in_mesh_local_kdl[1],
                                              radial_x_point_in_mesh_local_kdl[2]};

    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_x_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);
    // take the closest point to the current pose of the object
    KDL::Vector closest_point_to_x_point = tube_mesh_pose *
                                           KDL::Vector(closest_point[0],closest_point[1],closest_point[2]);

    // ----------------------- THIRD CLOSEST POINT
    //Find the closest cell to the radial tool point
    KDL::Vector radial_y_point_kdl = ring_pose *
                                     KDL::Vector(0., ring_radius, 0.f);
    KDL::Vector radial_y_point_in_mesh_local_kdl = tube_mesh_pose_inv*radial_y_point_kdl;

    double radial_y_point_in_mesh_local[3] = {radial_y_point_in_mesh_local_kdl[0],
                                              radial_y_point_in_mesh_local_kdl[1],
                                              radial_y_point_in_mesh_local_kdl[2]};

    cellLocator->Update();
    cellLocator->FindClosestPoint(radial_y_point_in_mesh_local, closest_point,
                                  cell_id, subId, closestPointDist2);
    // take the closest point to the current pose of the object
    KDL::Vector closest_point_to_y_point = tube_mesh_pose *
                                           KDL::Vector(closest_point[0],closest_point[1],closest_point[2]);


    // Find the vector from ring center to the corresponding closest point on
    // the thin mesh
    //KDL::Vector ring_center_to_cp = closest_point_to_center_point - ring_pose.p;

    // desired pose only when the ring is close to the wire.if it is too
    // far we don't want fixtures
    //if (ring_center_to_cp.Norm() < 5 * ring_radius) {

    // Desired position is one that puts the center of the ring on the
    // center of the mesh.

    //---------------------------------------------------------------------
    // Find the desired position of the ring assuming that the length
    // of the thin tube is negligible
    desired_ring_pose.p = closest_point_to_center_point;

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

    desired_ring_pose.M =
            KDL::Rotation(desired_x, desired_y, desired_z);

    // draw the connection lines for debug
    //line1_source->SetPoint1(radial_x_point_kdl[0],
    //                        radial_x_point_kdl[1],
    //                        radial_x_point_kdl[2]);
    //line1_source->SetPoint2(closest_point_to_x_point[0],
    //                        closest_point_to_x_point[1],
    //                        closest_point_to_x_point[2]);
    //line2_source->SetPoint1(radial_y_point_kdl[0],
    //                        radial_y_point_kdl[1],
    //                        radial_y_point_kdl[2]);
    //line2_source->SetPoint2(closest_point_to_y_point[0],
    //                        closest_point_to_y_point[1],
    //                        closest_point_to_y_point[2]);


}


//------------------------------------------------------------------------------
void TaskSteadyHand::UpdateRingColor() {

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

    ring_mesh[ring_in_action]->GetActor()->GetProperty()
            ->SetColor(colors.Orange[0],
                       colors.Orange[1]- 0.6*
                                         (error_ratio-0.3),
                       colors.Orange[2]);
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


void TaskSteadyHand::HapticsThread() {

    //----------------------------------------------
    // setting  up haptics

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();

    ros::Publisher pub_slave_desired[2];
    ros::Publisher pub_wrench_abs[2];

    //getting the name of the arms
    std::string param_name = "/atar/PSM1_DUMMY/tool_pose_desired";
    pub_slave_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            (param_name, 1);
    ROS_INFO("Will publish on %s", param_name.c_str());

    // make sure the masters are in wrench absolute orientation
    // assuming MTMR is always used
    std::string master_topic = "/dvrk/MTMR/set_wrench_body_orientation_absolute";
    pub_wrench_abs[0] = node->advertise<std_msgs::Bool>(master_topic.c_str(), 1);
    std_msgs::Bool wrench_body_orientation_absolute;
    wrench_body_orientation_absolute.data = 1;
    pub_wrench_abs[0].publish(wrench_body_orientation_absolute);
    ROS_INFO("Setting wrench_body_orientation_absolute on %s", master_topic.c_str());

    //getting the name of the arms
    param_name = "/atar/PSM2_DUMMY/tool_pose_desired";
    pub_slave_desired[1] = node->advertise<geometry_msgs::PoseStamped>
            (param_name, 1);
    ROS_INFO("Will publish on %s", param_name.c_str());

    // make sure the masters are in wrench absolute orientation
    // assuming MTMR is always used
    master_topic = "/dvrk/MTML/set_wrench_body_orientation_absolute";
    pub_wrench_abs[1] = node->advertise<std_msgs::Bool>(master_topic.c_str(), 1);
    pub_wrench_abs[1].publish(wrench_body_orientation_absolute);
    ROS_INFO("Setting wrench_body_orientation_absolute on %s", master_topic.c_str());


    ros::Rate loop_rate(haptic_loop_rate);
    ROS_INFO("The desired pose will be updated at '%f'",
             haptic_loop_rate);

    // publish ring poses (at a lower rate) for data analysis
    ros::Publisher pub_ring_desired, pub_ring_current;
    std::string ring_topic = "/atar/ring_pose_current";
    pub_ring_current = node->advertise<geometry_msgs::Pose>
            (ring_topic.c_str(), 10);
    ROS_INFO("Will publish on %s", ring_topic.c_str());

    ring_topic = "/atar/ring_pose_desired";
    pub_ring_desired = node->advertise<geometry_msgs::Pose>
            (ring_topic.c_str(), 10);
    ROS_INFO("Will publish on %s", ring_topic.c_str());
    int lower_freq_pub_counter = 0;

    // todo: what if this changes during the task executionn...
    KDL::Frame world_to_slave_tr[2];
    world_to_slave_tr[0] = slaves[0]->GetWorldToLocalTr();
    world_to_slave_tr[1] = slaves[1]->GetWorldToLocalTr();

    //---------------------------------------------
    // loop
    while (ros::ok())
    {
        slaves[0]->GetPoseWorld(tool_current_pose[0]);
        slaves[0]->GetGripper(gripper_angle[0]);
        slaves[1]->GetPoseWorld(tool_current_pose[1]);
        slaves[1]->GetGripper(gripper_angle[1]);

        KDL::Frame ring_pose_loc;

        //ring_pose = ring_mesh[ring_in_action]->GetPose();
        ring_pose_loc = ring_pose;

        KDL::Frame tr_to_desired_ring_pose, desired_ring_pose;
        KDL::Frame estimated_ring_pose_loc;

        // the pose of the ring is updated with the graphics frequency which
        // is too low for haptics. The good news is that if we assume that
        // the ring does not move relative to the forceps when gripped, we
        // can use the pose of the forceps that are updated fast and find the
        // ring_pose. When the ring is not gripped we do not need haptics but
        // to calculate the errors etc we still need to know the pose of the
        // ring the desired one.
        //         tool_to_ring_tr[0] = ring_pose * tool_current_pose[0].Inverse();

        if(gripper_in_contact[0])
            estimated_ring_pose_loc = tool_current_pose[0] *tool_to_ring_tr[0];
        else if (gripper_in_contact[1])
            estimated_ring_pose_loc = tool_current_pose[1]* tool_to_ring_tr[1];
        else
            estimated_ring_pose_loc = ring_pose_loc;

        // save in global for use in the other thread
        estimated_ring_pose = estimated_ring_pose_loc;

        // calculate the desired pose
        CalculatedDesiredRingPose(estimated_ring_pose_loc, desired_ring_pose);

        tr_to_desired_ring_pose.p = desired_ring_pose.p - estimated_ring_pose_loc.p;
        tr_to_desired_ring_pose.M = desired_ring_pose.M * estimated_ring_pose_loc.M
                .Inverse();
        //tr_to_desired_ring_pose = desired_ring_pose * estimated_ring_pose_loc.Inverse();

        // --------------- Soft start delta calculation
        // did we grasp the ring just now?
        if( (gripper_in_contact[0] & !gripper_in_contact_last[0]) ||
            (gripper_in_contact[1] & !gripper_in_contact_last[1]) )
            ac_soft_start_counter = 0;

        double soft_start_delta;
        if(ac_soft_start_counter < ac_soft_start_duration){
            soft_start_delta = double(ac_soft_start_counter)
                               /double(ac_soft_start_duration);
            ac_soft_start_counter++;
        }
        else
            soft_start_delta = 1.0;

        // --------------- Publish desired poses
        for (int n_arm = 0; n_arm < 2; ++n_arm) {

            if(gripper_in_contact[n_arm]) {

                // here find the desired tool pose if it is in contact with
                // the ring we add the displacement that would take the ring
                // to its desired pose to the current pose of the tool;
                tool_desired_pose[n_arm].p =
                        soft_start_delta * tr_to_desired_ring_pose.p +
                        tool_current_pose[n_arm].p;
                tool_desired_pose[n_arm].M =
                        tr_to_desired_ring_pose.M * tool_current_pose[n_arm].M;

            }
            else {
                tool_desired_pose[n_arm]= tool_current_pose[n_arm];
            }

            // convert to pose message
            geometry_msgs::PoseStamped pose_msg;
            KDL::Frame tool_desired_pose_in_slave_frame;
            tool_desired_pose_in_slave_frame =
                    world_to_slave_tr[n_arm]*tool_desired_pose[n_arm];

            tf::poseKDLToMsg(tool_desired_pose_in_slave_frame, pose_msg.pose);
            // fill the header
            pose_msg.header.frame_id = "/slave_frame";
            pose_msg.header.stamp = ros::Time::now();
            // publish
            pub_slave_desired[n_arm].publish(pose_msg);
        }


        lower_freq_pub_counter++;
        // publish the ring poses
        if(lower_freq_pub_counter==10){
            lower_freq_pub_counter = 0;
            pub_ring_current.publish(conversions::KDLFramePoseMsg(estimated_ring_pose_loc));
            pub_ring_desired.publish(conversions::KDLFramePoseMsg
                                             (desired_ring_pose));
        }
        //------------------------------------------------------------------
        // Calculate errors
        position_error_norm = tr_to_desired_ring_pose.p.Norm();
        KDL::Vector rpy;
        tr_to_desired_ring_pose.M.GetRPY(rpy[0],
                                         rpy[1],
                                         rpy[2]);
        orientation_error_norm = rpy.Norm();

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}

void TaskSteadyHand::CalculateAndSaveError() {

    double duration = (ros::Time::now() - start_time).toSec();
    double posit_error_avg = posit_error_sum/(double)sample_count;
    double orient_error_avg = orient_error_sum/(double)sample_count;

    double posit_error_avg_ideal = 0.001;
    double orient_error_avg_ideal = 0.30;
    double posit_error_max_ideal = 0.003;
    double orient_error_max_ideal = M_PI*30./180.;
    double duration_ideal = 65.0;

    // put a threshold on the values
    if (posit_error_avg < posit_error_avg_ideal)
        posit_error_avg = posit_error_avg_ideal;

    if (orient_error_avg < orient_error_avg_ideal)
        orient_error_avg = orient_error_avg_ideal;

    if (orient_error_max < orient_error_max_ideal)
        orient_error_max = orient_error_max_ideal;

    if (posit_error_max < posit_error_max_ideal)
        posit_error_max = posit_error_max_ideal;
    else if(posit_error_max > 0.004)
        posit_error_max = 1000.;

    if (duration < duration_ideal)
        duration = duration_ideal;

    double score = ( posit_error_avg_ideal/posit_error_avg
                     + posit_error_max_ideal/posit_error_max
                     + orient_error_max_ideal/orient_error_max
                     + duration_ideal/duration
                     + orient_error_avg_ideal/orient_error_avg)
                   * 100 / 5;

    // when the history gets full we start a new set
    if (score_history.size() == ring_num)
        ResetScoreHistory();

    score_history.push_back(score);
    score_history_colors.push_back(GetScoreColor(score));

    // update spheres' color
    for (int i = 0; i < score_history.size(); ++i) {
        score_sphere_actors[i]->GetProperty()->SetColor(score_history_colors[i]);

    }

    //ROS_INFO("posit_error_max: %f, score: %f", posit_error_max,
    //         posit_error_max_ideal/posit_error_max*100);
    //ROS_INFO("orient_error_max: %f, score: %f", orient_error_max,
    //         orient_error_max_ideal/orient_error_max*100);
    //ROS_INFO("duration: %f, score: %f", duration,duration_ideal/duration *100);
    //ROS_INFO("posit_error_avg: %f, score: %f", posit_error_avg,
    //         posit_error_avg_ideal/posit_error_avg*100);
    //ROS_INFO("orient_error_avg: %f, score: %f", orient_error_avg,
    //         orient_error_avg_ideal/orient_error_avg*100);
    ROS_INFO("User Feedback Score: %f", score);
    //ROS_INFO("  ");
}

double * TaskSteadyHand::GetScoreColor(const double score) {

    //decide the color
    if(score > 90){
        return colors.Green;
    }
    else if (score > 80)
        return colors.Gold;
    else if(score > 60)
        return colors.OrangeDark;
    else
        return colors.Red;

}

void TaskSteadyHand::ResetOnGoingEvaluation() {
    posit_error_sum = 0.0;
    posit_error_max = 0.0;
    orient_error_max = 0.0;
    orient_error_sum = 0.0;
    sample_count = 0;
}

void TaskSteadyHand::ResetScoreHistory() {
    score_history.clear();
    score_history_colors.clear();

    // reset colors to gray
    for (int i = 0; i < ring_num; ++i) {
        score_sphere_actors[i]->GetProperty()->SetColor(colors.Gray);

    }

}


//void TaskSteadyHand::StepPhysics() {
//    ///-----stepsimulation_start-----
//    double time_step = (ros::Time::now() - time_last).toSec();
//    //std::cout << "time_step: " << time_step << std::endl;
//    // simulation seems more realistic when time_step is halved right now!
//    dynamics_world->stepSimulation(btScalar(time_step), 100, 1/256.f);
//    time_last = ros::Time::now();
//}


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
        rods[i]->SetKinematicPose(arm_pose);

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
        rods[i]->SetKinematicPose(arm_pose);
    }
}

TaskSteadyHand::~TaskSteadyHand() {

    delete slaves[0];
    delete slaves[1];
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
