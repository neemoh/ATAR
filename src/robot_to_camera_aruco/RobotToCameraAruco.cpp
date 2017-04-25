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

    // declaration of target markers for calib2
    for (uint i=0; i<num_calib_points; i++){
        if(i<num_calib_points/2)
            target.push_back(cv::Point3d((1+i)*(aruco_marker_length_in_meters+aruco_marker_separation_in_meters), aruco_marker_length_in_meters, 0.0));
        else
            target.push_back(cv::Point3d((1+i)*(aruco_marker_length_in_meters+aruco_marker_separation_in_meters), 2*aruco_marker_length_in_meters+aruco_marker_separation_in_meters, 0.0));
    }

    cv::projectPoints(target, task_frame_to_cam_rvec[cam_id],
                      task_frame_to_cam_tvec[cam_id], camera_intrinsics.camMatrix,
                      camera_intrinsics.distCoeffs, calib_points_screen);
};


//-----------------------------------------------------------------------------------
// SetupROS
//-----------------------------------------------------------------------------------

void RobotToCameraAruco::SetupROS() {
    bool all_required_params_found = true;

    n.param<double>("frequency", ros_freq, 25);

    n.param<int>("cam_id", cam_id, 0);
    if(cam_id==1)
        ROS_INFO("Using right camera.");
    else if (cam_id == 0)
        ROS_INFO("Using Left camera.");
    else
        ROS_ERROR("cam_is param can be set as either 0 (for left) or 1 (for "
                          "right)");

    if (n.getParam("visual_axis_length", visual_axis_length))
        ROS_INFO_STREAM(std::string("Using parameter ")
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

    // if the task_frame_to_robot_frame parameter is present from a previous calibration we can
    // show it
    std::string arm_name;
    (n.getParam("robot_arm_name", arm_name));
    std::stringstream param_name;
    param_name << std::string("/calibrations/task_frame_to_") << arm_name << "_frame";
    std::vector<double> task_frame_to_robot_frame_vec7(7, 0.0);
    if(n.getParam(param_name.str(), task_frame_to_robot_frame_vec7)){

        task_frame_to_robot_frame_param_present = true;
        //Convert and save
        conversions::VectorToKDLFrame(task_frame_to_robot_frame_vec7, task_frame_to_robot_frame);
        std::cout << param_name.str() << " parameter was received as: "
                  << task_frame_to_robot_frame_vec7 << std::endl;
    }



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
    std::string image_topic_name;
    if(cam_id)
        image_topic_name = "/camera/right/image_color";
    else
        image_topic_name = "/camera/left/image_color";

    ROS_INFO("Camera images will be read from topic '%s'", image_topic_name
            .c_str());
    camera_image_subscriber = it->subscribe(
            image_topic_name, 1, &RobotToCameraAruco::CameraImageCallback,
            this);


    // a topic name is required for the camera pose
    std::string cam_pose_topic_name;
    if (n.getParam("left_cam_pose_topic_name", cam_pose_topic_name)) {

        // if the topic name is found, check if something is being published
        // on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            cam_pose_topic_name, ros::Duration(1))) {
            ROS_WARN("Topic '%s' is not publishing.",
                     cam_pose_topic_name.c_str());
        } else
            ROS_INFO("Reading left camera pose from topic '%s'",
                     cam_pose_topic_name.c_str());
    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("left_cam_pose_topic_name").c_str());
        all_required_params_found = false;
    }

    // register camera pose subscriber
    left_camera_pose_subscriber =
        n.subscribe(cam_pose_topic_name, 10,
                    &RobotToCameraAruco::LeftCameraPoseCallback, this);


    // a topic name is required for the camera pose
    if (n.getParam("right_cam_pose_topic_name", cam_pose_topic_name)) {

        // if the topic name is found, check if something is being published
        // on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            cam_pose_topic_name, ros::Duration(1))) {
            ROS_WARN("Topic '%s' is not publishing.",
                     cam_pose_topic_name.c_str());
        } else
            ROS_INFO("Reading right camera pose from topic '%s'",
                     cam_pose_topic_name.c_str());
    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("right_cam_pose_topic_name").c_str());
        all_required_params_found = false;
    }

    // register camera pose subscriber
    right_camera_pose_subscriber =
        n.subscribe(cam_pose_topic_name, 10,
                    &RobotToCameraAruco::RightCameraPoseCallback, this);




    // a topic name is required for the robot pose
    std::stringstream robot_pose_topic_name;
    robot_pose_topic_name << std::string("/dvrk/")
            << arm_name << "/position_cartesian_current";
        // if the topic name is found, check if something is being published
        // on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
            robot_pose_topic_name.str().c_str(), ros::Duration(1))) {
            ROS_ERROR("Topic '%s' is not publishing.",
                      robot_pose_topic_name.str().c_str());
                          all_required_params_found = false;
        } else{
            ROS_INFO("Reading robot pose from topic '%s'",
                     robot_pose_topic_name.str().c_str());
    }
    // register camera pose subscriber
    robot_pose_subscriber =
        n.subscribe(robot_pose_topic_name.str(), 10,
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
    cv::projectPoints(axisPoints, task_frame_to_cam_rvec[cam_id],
    task_frame_to_cam_tvec[cam_id],
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

//    /////// FOR DEBUG
//    measured_points.push_back(Eigen::Vector3d(0.10784, -0.0464285, -0.0861506));
//    measured_points.push_back(Eigen::Vector3d(0.137249, -0.0465126, -0.0856462));
//    measured_points.push_back(Eigen::Vector3d(0.155377, -0.0470039, -0.0853909));
//    measured_points.push_back(Eigen::Vector3d(0.0987962, -0.0195716, -0.0857134));
//    measured_points.push_back(Eigen::Vector3d(0.0984003, -0.00750025, -0.0864288));
//    measured_points.push_back(Eigen::Vector3d(0.098532, -0.00016544, -0.0860863));
//    //


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
    for (uint i=0; i<num_calib_points; i++){
        cv::circle(img, calib_points_screen[i], 6, cv::Scalar(255, 255, 0),
                   1, CV_AA);
    }
    // draw actual target with a different colour
    cv::circle(img, calib_points_screen[meas_points.size()], 6, cv::Scalar(0, 0, 255),
               1, CV_AA);
    instructions = oss.str();
}

void RobotToCameraAruco::Calib2SaveCalibrationPoint() {

    size_t num_p = meas_points.size();
    if (num_p < num_calib_points) {
        // save the position of the end effector
        meas_points.push_back(
            Eigen::Vector3d(
                tool_pose_in_robot_frame.p[0], tool_pose_in_robot_frame.p[1],
                tool_pose_in_robot_frame.p[2]));
    }
    if (num_p == num_calib_points) {
        Calib2CalculateTransformation();
    }
}

void RobotToCameraAruco::Calib2CalculateTransformation() {

    size_t num_p = target.size();

    for (uint i = 0; i < num_p; i++) {
        meas_points_mat.col(i) = meas_points[i];
        target_mat(0,i) = target[i].x;
        target_mat(1,i) = target[i].y;
        target_mat(2,i) = target[i].z;
    }

    auto temp = Eigen::umeyama(target_mat, meas_points_mat, 0);

    // converting Eigen::Matrix into KDL::Frame
    cv::Matx33d rot_mat;
    for (uint i = 0; i < 3; i++) {
        rot_mat(i, 0) = temp(i, 0);
        rot_mat(i, 1) = temp(i, 1);
        rot_mat(i, 2) = temp(i, 2);
        task_frame_to_robot_frame.p[i] = temp(i, 3);
    }
    conversions::Matx33dToKdlRot(rot_mat, task_frame_to_robot_frame.M);
    ROS_INFO_STREAM("  -> Board To PSM Transformation: \n" << temp << std::endl);
    calib_finished = true;
}


void RobotToCameraAruco::SetROSParameters() {

    std::string arm_name;
    (n.getParam("robot_arm_name", arm_name));

    std::stringstream param_name;
    param_name << std::string("/calibrations/task_frame_to_") << arm_name << "_frame";

    std::vector<double> vec7(7, 0.0);
    conversions::KDLFrameToVector(task_frame_to_robot_frame, vec7);
    n.setParam(param_name.str(), vec7);
    std::cout<< param_name.str()<< " parameter was set to: "<< vec7 << std::endl;


    param_name.str("");
    param_name << std::string("/calibrations/task_frame_to_left_cam_frame");
    conversions::KDLFrameToVector(task_frame_to_cam_frame[0], vec7);
    n.setParam(param_name.str(), vec7);
    std::cout<< param_name.str()<< " parameter was set to: "<< vec7 << std::endl;


    param_name.str("");
    param_name << std::string("/calibrations/task_frame_to_right_cam_frame");
    conversions::KDLFrameToVector(task_frame_to_cam_frame[1], vec7);
    n.setParam(param_name.str(), vec7);
    std::cout<< param_name.str()<< " parameter was set to: "<< vec7 << std::endl;


    // print the pose of board_to_cam
    auto camera_frame_to_robot_frame = task_frame_to_robot_frame *
    task_frame_to_cam_frame[cam_id].Inverse();
    conversions::KDLFrameToVector(camera_frame_to_robot_frame, vec7);
    std::cout<< " camera_frame_to_robot_frame is: "<< vec7 << std::endl;


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

void RobotToCameraAruco::LeftCameraPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // converting to frame and rvec/tvec
    tf::poseMsgToKDL(msg->pose, task_frame_to_cam_frame[0]);
    conversions::KDLFrameToRvectvec(task_frame_to_cam_frame[0],
                                    task_frame_to_cam_rvec[0],
                                    task_frame_to_cam_tvec[0]);
}

void RobotToCameraAruco::RightCameraPoseCallback(
        const geometry_msgs::PoseStamped::ConstPtr &msg) {

    // converting to frame and rvec/tvec
    tf::poseMsgToKDL(msg->pose, task_frame_to_cam_frame[1]);
    conversions::KDLFrameToRvectvec(task_frame_to_cam_frame[1],
                                    task_frame_to_cam_rvec[1],
                                    task_frame_to_cam_tvec[1]);
}

cv::Mat &RobotToCameraAruco::Image(ros::Duration timeout) {
    ros::Rate loop_rate(1);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while (image_msg.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new Image received.");
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