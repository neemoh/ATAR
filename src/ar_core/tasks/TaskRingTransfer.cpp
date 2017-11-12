//
// Created by nima on 21/07/17.
//

#include "TaskRingTransfer.h"
#include <custom_conversions/Conversions.h>
#include <boost/thread/thread.hpp>
#include <vtkAxesActor.h>


TaskRingTransfer::TaskRingTransfer(ros::NodeHandlePtr n)
        :
        SimTask(n, 100),
        time_last(ros::Time::now())
{

    int n_views = 3;
    bool one_window_per_view = false;
    bool borders_off  = true;
    std::vector<int> view_resolution = {640, 480};
    std::vector<int> window_positions={1280, 0};
    // the only needed argument to construct a Renderer if the nodehandle ptr
    // The rest have default values.
    graphics = std::make_unique<Rendering>(n, view_resolution, false, n_views,
                                           one_window_per_view, borders_off,window_positions);

    // Define a master manipulator
    //    master = new Manipulator(nh, "/sigma7/sigma0", "/pose", "/gripper_angle");
    master = new Manipulator(nh, "/dvrk/PSM1_DUMMY",
                             "/position_cartesian_current",
                             "/gripper_position_current");
    // -------------------------------------------------------------------------
    // Create a cube for the board
    SimObject *board;
    {
        board_dimensions[0] = 0.14;
        board_dimensions[1] = 0.12;
        board_dimensions[2] = 0.01;

        double friction = 0.5;

        KDL::Frame pose;
        pose.p = KDL::Vector(
                - 0.01 + board_dimensions[0]/2,  board_dimensions[1]/2,
                -board_dimensions[2]/2);

        std::vector<double> dim = {
                board_dimensions[0], board_dimensions[1], board_dimensions[2]
        };
        board = new SimObject(ObjectShape::BOX, ObjectType::DYNAMIC, dim, pose,
                              0.0, friction);
        //    board->GetActor()->GetProperty()->SetOpacity(0.05);
        board->GetActor()->GetProperty()->SetColor(0.6, 0.5, 0.5);

        AddSimObjectToTask(board);
    }
    // -------------------------------------------------------------------------
    // static floor
    // always add a floor in under the workspace of your workd to prevent
    // objects falling too far and mess things up.
    std::vector<double> dims = {0., 0., 1., -0.5};
    SimObject* floor= new SimObject(ObjectShape::STATICPLANE, dims);
    dynamics_world->addRigidBody(floor->GetBody());

    // -------------------------------------------------------------------------
    // Create destination rods
    int cols = 4;
    int rows = 1;
    SimObject *cylinders[cols * rows];

    {
        float friction = 2.2;
        for (int i = 0; i < rows; ++i) {

            for (int j = 0; j < cols; ++j) {

                std::vector<double> dim = {0.002, 0.035};

                KDL::Frame pose(KDL::Rotation::Quaternion(0, 0.70711,
                                                          0.70711, 0.0),
                                KDL::Vector( 0.02 + (double) j * 0.028, 0.09,
                                             dim[1] / 2));

                cylinders[i * rows + j] =
                        new SimObject(ObjectShape::CYLINDER,
                                      ObjectType::DYNAMIC, dim, pose, 0.0,
                                      friction);

                auto ratio = (float) j / (float) cols;
                cylinders[i * rows + j]->GetActor()->GetProperty()->SetColor(
                        0.9 - 0.3 * ratio, 0.4, 0.4 + 0.4 * ratio);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecular(0.8);
                cylinders[i*rows+j]->GetActor()->GetProperty()
                        ->SetSpecularPower(50);
                AddSimObjectToTask(cylinders[i * rows + j]);
            }
        }
    }

    // -------------------------------------------------------------------------
    // ROD
    SimObject* rod;
    {
        std::vector<double> rod_dim = {0.002, 0.1};
        KDL::Frame pose(KDL::Rotation::Quaternion(0.70711, 0.70711, 0.0, 0.0),
                        KDL::Vector(0.10, 0.03, 0.03));
        rod = new SimObject(ObjectShape::CYLINDER, ObjectType::DYNAMIC,
                                  rod_dim, pose);
        rod->GetActor()->GetProperty()->SetColor(0.3, 0.3, 0.3);
        AddSimObjectToTask(rod);

    }
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    //// Create smallRING meshes
    size_t n_rings = 4;
    SimObject *rings[n_rings];
    {
        float density = 50000;
        float friction = 5;
        for (int l = 0; l < n_rings; ++l) {

            KDL::Frame pose(KDL::Rotation::Quaternion(0.70711, 0.70711, 0.0, 0.0),
                            KDL::Vector(0.06 + (double) l * 0.01, 0.03, 0.03));
            std::vector<double> dim; // not used

            rings[l] = new
                    SimObject(ObjectShape::MESH, ObjectType::DYNAMIC,
                              RESOURCES_DIRECTORY+"/mesh/task_Hook_ring_D2cm_D5mm.obj",
                              pose, density, friction);
            rings[l]->GetActor()->GetProperty()->SetColor(0., 0.5, 0.6);
            AddSimObjectToTask(rings[l]);

        }
    }
// -------------------------------------------------------------------------
    // Create kinematic gripper
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
        gripper_link_dims =
                {{0.003, 0.003, 0.005}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.007}
                        , {0.004, 0.001, 0.007}};

        grippers[0] = new SimFiveLinkGripper(gripper_link_dims);
        AddSimMechanismToTask(grippers[0]);
        std::vector<std::vector<double>> gripper_3link_dims =
                {{0.002, 0.002, 0.005}
                        , {0.004, 0.001, 0.009}
                        , {0.004, 0.001, 0.009}};

    }


    // -------------------------------------------------------------------------
    //// Create hook mesh
    {
        KDL::Frame pose(KDL::Rotation::Quaternion(0.7, 0, 0.7, 0),
                        KDL::Vector(0.09, 0.07, 0.08));
        std::vector<double> dim; // not used
        float density = 50000;

        hook_mesh = new
                SimObject(ObjectShape::MESH, ObjectType::KINEMATIC,
                          RESOURCES_DIRECTORY+"/mesh/task_hook_hook.obj", pose,
                          density, 0);
        hook_mesh->GetActor()->GetProperty()->SetColor(1., 1.0, 1.0);
        AddSimObjectToTask(hook_mesh);
    }

    // -------------------------------------------------------------------------
    // FRAMES
    vtkSmartPointer<vtkAxesActor> task_coordinate_axes =
            vtkSmartPointer<vtkAxesActor>::New();

    task_coordinate_axes->SetXAxisLabelText("");
    task_coordinate_axes->SetYAxisLabelText("");
    task_coordinate_axes->SetZAxisLabelText("");
    task_coordinate_axes->SetTotalLength(0.01, 0.01, 0.01);
    task_coordinate_axes->SetShaftType(vtkAxesActor::CYLINDER_SHAFT);
    bool show_ref_frames = 1;
    if(show_ref_frames)
        graphics->AddActorToScene(task_coordinate_axes);

    
    
};

//------------------------------------------------------------------------------
void TaskRingTransfer::TaskLoop() {


    //-------------------------------- UPDATE RIGHT GRIPPER
    KDL::Frame grpr_right_pose;
    master->GetPoseWorld(grpr_right_pose);
    // map gripper value to an angle

    double grip_posit;
    master->GetGripper(grip_posit);
    double theta_min=0*M_PI/180;
    double theta_max=40*M_PI/180;
    double grip_angle = theta_max*(grip_posit+0.5)/1.55;
    if(grip_angle<theta_min)
        grip_angle=theta_min;

    grippers[0]->SetPoseAndJawAngle(grpr_right_pose, grip_angle);

    //Update the pose of hook
    {
        KDL::Frame tool_pose;
        KDL::Frame local_tool_transform;

        // locally transform the tool if needed
        local_tool_transform.p = KDL::Vector(0.0, 0.0, 0);
        local_tool_transform.M.DoRotY(0);
        tool_pose = tool_pose * local_tool_transform;

        //KDL::Vector box_posit = tool_pose * KDL::Vector(0.0, 0.0, 0);

        double x, y, z, w;
        tool_pose.M.GetQuaternion(x, y, z, w);
        double box_pose[7] = {tool_pose.p.x(), tool_pose.p.y(), tool_pose.p.z(),
                              x, y, z, w};
        hook_mesh->SetKinematicPose(box_pose);

    }



}


custom_msgs::TaskState TaskRingTransfer::GetTaskStateMsg() {
    custom_msgs::TaskState task_state_msg;
    return task_state_msg;
}

void TaskRingTransfer::ResetTask() {
    ROS_INFO("Resetting the task.");

}

void TaskRingTransfer::ResetCurrentAcquisition() {
    ROS_INFO("Resetting current acquisition.");

}


void TaskRingTransfer::HapticsThread() {

    ros::Publisher pub_desired[2];

    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
    pub_desired[0] = node->advertise<geometry_msgs::PoseStamped>
            ("/PSM1/tool_pose_desired", 10);
//    if(bimanual)
//        pub_desired[1] = node->advertise<geometry_msgs::PoseStamped>
//                ("/PSM2/tool_pose_desired", 10);

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

//        CalculatedDesiredToolPose();

//        // publish desired poses
//        for (int n_arm = 0; n_arm < 1+ int(bimanual); ++n_arm) {
//
//            // convert to pose message
//            geometry_msgs::PoseStamped pose_msg;
//            tf::poseKDLToMsg(tool_desired_pose_kdl[n_arm], pose_msg.pose);
//            // fill the header
//            pose_msg.header.frame_id = "/task_space";
//            pose_msg.header.stamp = ros::Time::now();
//            // publish
//            pub_desired[n_arm].publish(pose_msg);
//        }

        ros::spinOnce();
        loop_rate.sleep();
        boost::this_thread::interruption_point();
    }
}


TaskRingTransfer::~TaskRingTransfer() {

    delete master;

}


