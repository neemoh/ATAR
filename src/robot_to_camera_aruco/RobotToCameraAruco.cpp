/*
 * Calibrator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#include "RobotToCameraAruco.hpp"
#include "utils/Colors.hpp"
#include "utils/Conversions.hpp"
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/flann.hpp>

#include <pwd.h>

RobotToCameraAruco::RobotToCameraAruco(std::string node_name)
        : calib_finished(true), n(node_name) {

    //    message = "Point at point 1 and press 'space' to start calibration";
    it = new image_transport::ImageTransport(n);

    SetupROS();

    // make 3 points at the corner for calibration
    calib_points_in_board_frame.push_back(cv::Point3f(0.0, 0.0, 0.0));
    calib_points_in_board_frame.push_back(cv::Point3f(0.07, 0.0, 0.0));
    calib_points_in_board_frame.push_back(cv::Point3f(0.0, 0.04, 0.0));

    target.push_back(cv::Point3d(cvflann::rand_double(RobotToCameraAruco::length_x, 0), cvflann::rand_double(RobotToCameraAruco::length_y, 0), 0.0));

};


//-----------------------------------------------------------------------------------
// SetupROS
//-----------------------------------------------------------------------------------

void RobotToCameraAruco::SetupROS() {
    bool all_required_params_found = true;

    n.param<double>("frequency", ros_freq, 25);

    if (n.getParam("visual_axis_length", visual_axis_length))
        ROS_INFO_STREAM("Using parameter "
                                << n.resolveName("visual_axis_length").c_str()
                                << " with value " << visual_axis_length);

    if (!n.getParam("number_of_calibration_points", num_calib_points))
        ROS_INFO_STREAM("Parameter"
                                << n.resolveName("number_of_calibration_points").c_str()
                                << ". Default value is used: "
                                << num_calib_points);
    else
        ROS_INFO_STREAM("Using parameter "
                                << n.resolveName("number_of_calibration_points").c_str()
                                << "with value " << num_calib_points
                                << " points per axis. ");
    // ------- load the intrinsic calibration files
    // get home directory
    struct passwd *pw = getpwuid(getuid());
    const char *home_dir = pw->pw_dir;
    std::string cam_name;
    if (n.getParam("cam_name", cam_name)) {

        std::stringstream path;
        path << std::string(home_dir) << std::string("/.ros/camera_info/")
             << cam_name << "_intrinsics.xml";
        ReadCameraParameters(path.str(), camera_intrinsics);
    } else
        ROS_ERROR("%s Parameter '%s' is required. Place the intrinsic calibration "
                          "file of each camera in ~/.ros/camera_info/ named as <cam_name>_intrinsics.xml",
                  ros::this_node::getName().c_str(), n.resolveName("cam_name").c_str());


    // a topic name is required for the images

    std::string image_transport_namespace;
    if (n.getParam("image_transport_namespace", image_transport_namespace)) {

        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<sensor_msgs::Image>( image_transport_namespace, ros::Duration(5))) {
            ROS_ERROR("Topic '%s' is not publishing.", image_transport_namespace.c_str());
            all_required_params_found = false;
        }
        else
            ROS_INFO("Reading camera images from transport '%s'", image_transport_namespace.c_str());

    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("image_transport_namespace").c_str());
        all_required_params_found = false;
    }

    // register image transport subscriber
    camera_image_subscriber = it->subscribe(
            image_transport_namespace, 1, &RobotToCameraAruco::CameraImageCallback, this);



    // a topic name is required for the camera pose
    std::string cam_pose_topic_name;
    if (n.getParam("cam_pose_topic_name", cam_pose_topic_name)) {

        // if the topic name is found, check if something is being published
        // on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                cam_pose_topic_name, ros::Duration(1))) {
            ROS_WARN("Topic '%s' is not publishing.",
                     cam_pose_topic_name.c_str());
            //                      all_required_params_found = false;
        } else
            ROS_INFO("Reading camera pose from topic '%s'",
                     cam_pose_topic_name.c_str());
    } else {
        ROS_ERROR("Parameter '%s' is required.",
                  n.resolveName("cam_pose_topic_name").c_str());
        all_required_params_found = false;
    }

    // register camera pose subscriber
    camera_pose_subscriber =
            n.subscribe(cam_pose_topic_name, 10,
                        &RobotToCameraAruco::CameraPoseCallback, this);

    // a topic name is required for the robot pose
    std::string robot_pose_topic_name;
    if (n.getParam("robot_pose_topic_name", robot_pose_topic_name)) {

        // if the topic name is found, check if something is being published
        // on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                robot_pose_topic_name, ros::Duration(1))) {
            ROS_ERROR("Topic '%s' is not publishing.",
                      robot_pose_topic_name.c_str());
            //              all_required_params_found = false;
        } else
            ROS_INFO("Reading robot pose from topic '%s'",
                     robot_pose_topic_name.c_str());
    } else {
        ROS_ERROR("Parameter '%s' is required.",
                  n.resolveName("robot_pose_topic_name").c_str());
        all_required_params_found = false;
    }

    // register camera pose subscriber
    robot_pose_subscriber =
            n.subscribe(robot_pose_topic_name, 10,
                        &RobotToCameraAruco::RobotPoseCallback, this);

    if (!all_required_params_found)
        throw std::runtime_error("ERROR: some required topics are not set");
}


void RobotToCameraAruco::Calib1DrawCalibrationAxis(cv::String &instructions,
                                                   cv::Mat img) {

    std::ostringstream oss;
    auto n_points = measured_points.size();

    // project the points for both axes
    std::vector<cv::Point3f> axisPoints{
            cv::Point3f(0, 0, 0), cv::Point3f(1, 0, 0) * visual_axis_length,
            cv::Point3f(0, 1, 0) * visual_axis_length,
    };

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, task_frame_to_cam_rvec, task_frame_to_cam_tvec,
                      camera_intrinsics.camMatrix,
                      camera_intrinsics.distCoeffs, imagePoints);

    if (n_points < num_calib_points/2) {
        // we are still doing the first axis (x)
        oss << "Take " << (num_calib_points/2 - n_points)
            << " points from the x axis axis.";
        // draw x axis
        cv::line(img, imagePoints[0], imagePoints[1], Colors::Blue, 2, CV_AA);

    } else if (n_points < num_calib_points) {
        // we are at the second axis (y)
        oss << "Take " << (num_calib_points - n_points)
            << " points from the y axis axis.";

        // draw y axis
        cv::line(img, imagePoints[0], imagePoints[2], Colors::Green, 2,
                 CV_AA);
    } else {
        // we are done
        oss << "Received all the points.";
    }

    instructions = oss.str();
}


void RobotToCameraAruco::Calib1SaveCalibrationPoint() {

    size_t n_points = measured_points.size();

    if (n_points < num_calib_points) {
        measured_points.push_back(
                Eigen::Vector3d(tool_pose_in_robot_frame.p[0], tool_pose_in_robot_frame.p[1]
                        , tool_pose_in_robot_frame.p[2]));
        std::cout << "Added point <" << n_points + 1 << "> : " << tool_pose_in_robot_frame.p[0]
           << " " << tool_pose_in_robot_frame.p[1] << " " << tool_pose_in_robot_frame.p[2] << std::endl;
    }

    // if we got all the points find the transformation
    if (measured_points.size() == num_calib_points) {
        ROS_INFO("Finding the transformation...");
        // got all the points, find the transformation
        Calib1CalculateTransformation(measured_points, task_frame_to_robot_frame);
        // set the status as done
        calib_finished = true;
    }
}


std::pair<Eigen::Vector3d, Eigen::Vector3d>
RobotToCameraAruco::FindAxis(Eigen::MatrixXd axis_points) {

    auto n_points = axis_points.rows();

    // convert to std vector
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < n_points; ++i)
        points.push_back(axis_points.row(i));

    // fit the points and normalize
    auto axis_pair = FitLineToPoints(points);

    Eigen::Vector3d axis = axis_pair.second;
    axis.normalize();

    // finding the correct direction of the axis
    if (axis.dot(axis_points.row(1) - axis_points.row(0)) < 0) {
        axis = -axis;
        std::cout << "flipped axis" << std::endl;
    }
    return std::make_pair(axis_pair.first, axis);
}

void RobotToCameraAruco::Calib1CalculateTransformation(
        const std::vector<Eigen::Vector3d> axis_points, KDL::Frame & transformation) {

    auto n_points = axis_points.size();
    auto n_points_per_axis = (n_points) / 2;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> points(n_points, 3);
    for (uint i = 0; i < n_points; i++)
        points.row(i) = axis_points[i];

    // ---------------  fit plane to the points
    auto origin_axis_pair = FitPlaneToPoints(axis_points);
    Eigen::Vector3d plane_origin = origin_axis_pair.first;
    Eigen::Vector3d plane_normal = origin_axis_pair.second;

    // --------------- project the points on the plane
    Eigen::MatrixXd centered = points.rowwise() - plane_origin.transpose();

    // the dot product of the vector from each point to center and the plane
    // normal
    auto dist = centered * plane_normal;

    Eigen::MatrixXd projection_vectors(n_points, 3);

    for (uint iter = 0; iter < n_points; iter++)
        projection_vectors.row(iter) = dist(iter) * plane_normal;

    Eigen::MatrixXd points_projected = points - projection_vectors;

    // ---------------  fit the x axis
    auto x_axis_pair = FindAxis(points_projected.topRows(n_points_per_axis));
    Eigen::Vector3d x_axis_point = x_axis_pair.first;
    Eigen::Vector3d x_axis = x_axis_pair.second;

    //  --------------- fit the y axis
    auto y_axis_pair =
            FindAxis(points_projected.bottomRows(n_points_per_axis));
    Eigen::Vector3d y_axis_point = y_axis_pair.first;
    Eigen::Vector3d y_axis = y_axis_pair.second;

    //  --------------- find the z axis
    Eigen::Vector3d z_axis = x_axis.cross(y_axis);
    z_axis.normalize();

    // make sure y is normal to xz plane
    y_axis = z_axis.cross(x_axis);

    // Find the intersection of x and y
    // solving as a least-square problem Rp=q
    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd R =
            (I - x_axis * x_axis.transpose()) + (I - y_axis * y_axis.transpose());

    Eigen::VectorXd q = (I - x_axis * x_axis.transpose()) * x_axis_point +
                        (I - y_axis * y_axis.transpose()) * y_axis_point;

    Eigen::VectorXd origin =
            R.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(q);

    // convert to opencv matrix and vector
    // Todo use frameM directly
    cv::Matx33d rot_mat;
    for (uint i = 0; i < 3; i++) {
        rot_mat(i, 0) = x_axis(i);
        rot_mat(i, 1) = y_axis(i);
        rot_mat(i, 2) = z_axis(i);
        transformation.p[i] = origin(i);
    }
    conversions::Matx33dToKdlRot(rot_mat, transformation.M);

}


void RobotToCameraAruco::Calib2DrawTarget(cv::String &instructions, cv::Mat img) {

    std::ostringstream oss;
    oss << "Take the tool tip to the target point then press s";

    std::vector<double> target_point(3);

    target_point[0]=target[target.size()-1].x;
    target_point[1]=target[target.size()-1].y;
    target_point[2]=target[target.size()-1].z;
    cv::projectPoints(target_point, task_frame_to_cam_rvec,
                      task_frame_to_cam_tvec, camera_intrinsics.camMatrix,
                      camera_intrinsics.distCoeffs, calib_points_screen);

    cv::circle(img, calib_points_screen[0], 6, cv::Scalar(255, 255, 0),
               1, CV_AA);
    instructions = oss.str();
}

void RobotToCameraAruco::Calib2SaveCalibrationPoint() {

    size_t num_p = target.size();
    if (num_p < num_calib_points) {
        // save the position of the end effector
        meas_points.push_back(
                Eigen::Vector3d(
                        tool_pose_in_robot_frame.p[0], tool_pose_in_robot_frame.p[1],
                        tool_pose_in_robot_frame.p[2]));
        target.push_back(cv::Point3d(cvflann::rand_double(length_x, 0), cvflann::rand_double(length_y, 0), 0.0));
    }
    if (num_p == num_calib_points) {
        Calib2CalculateTransformation();
    }
}

void RobotToCameraAruco::Calib2CalculateTransformation() {

    size_t num_p = target.size();
    cv::Matx33d rot_temp;
    cv::Rodrigues(task_frame_to_cam_rvec, rot_temp);
    cv::Point3d temp(task_frame_to_cam_tvec);

    for (uint i = 0; i < num_p; i++) {
        auto camera_points = rot_temp * (target[i] - temp);     // before was "(rot_temp * targe[i])+temp", but this should be correct.
        points_on_camera.push_back(Eigen::Vector3d(camera_points.x, camera_points.y, camera_points.z));
        meas_points_mat.col(i) = meas_points[i];
        points_on_camera_mat.col(i) = points_on_camera[i];
    }

    camera_to_robot = Eigen::umeyama(points_on_camera_mat, meas_points_mat, 0);
    ROS_INFO_STREAM("  -> Camera To PSM Transformation: \n" << camera_to_robot << std::endl);
    calib_finished = true;

}




void RobotToCameraAruco::SetTaskFrameToRobotFrameParam() {

    std::vector<double> task_frame_to_robot_frame_vec7(7, 0.0);

    conversions::KDLFrameToVector(task_frame_to_robot_frame, task_frame_to_robot_frame_vec7);
    n.setParam("task_frame_to_robot_frame", task_frame_to_robot_frame_vec7);
    std::cout << "The task_frame_to_robot_frame parameter was set to: "
              << task_frame_to_robot_frame_vec7 << std::endl;

    // print the pose of board_to_cam
    geometry_msgs::Pose temp_pose;
    tf::poseKDLToMsg(task_frame_in_cam_frame, temp_pose);
    std::cout << "  -> task_frame_in_cam_frame: \n" << temp_pose << std::endl;

    tf::poseKDLToMsg(task_frame_to_robot_frame, temp_pose);
    std::cout << "  -> task_frame_to_robot_frame: \n" << temp_pose << std::endl;
}

void RobotToCameraAruco::Reset() {
    calib_finished = false;
}

void RobotToCameraAruco::ReadCameraParameters(const std::string file_path,
                                           CameraIntrinsics & camera) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> camera.camMatrix;
    fs["distortion_coefficients"] >> camera.distCoeffs;

    // check if we got osomething
    if(camera.distCoeffs.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(camera.camMatrix.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}

void RobotToCameraAruco::CameraImageCallback(
        const sensor_msgs::ImageConstPtr &msg) {
    try {
        image_msg = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }
}

void RobotToCameraAruco::CameraPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // converting to frame and rvec/tvec
    tf::poseMsgToKDL(msg->pose, task_frame_in_cam_frame);
    conversions::KDLFrameToRvectvec(task_frame_in_cam_frame, task_frame_to_cam_rvec,
                                    task_frame_to_cam_tvec);
}

cv::Mat &RobotToCameraAruco::Image(ros::Duration timeout) {
    ros::Rate loop_rate(0.2);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while (image_msg.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout whilst waiting for a new image from the "
                             "image topic. "
                             "Is the camera publishing?");
        }
    }

    return image_msg;
}

void RobotToCameraAruco::RobotPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // save the pose in a kdl frame
    tf::poseMsgToKDL(msg->pose, tool_pose_in_robot_frame);
}

std::ostream &operator<<(std::ostream &out, const std::vector<double> &vect) {
    for (unsigned int iter = 0; iter < vect.size(); ++iter) {
        out << "[" << iter << "]: " << vect.at(iter) << "\t";
    }

    return out;
}
