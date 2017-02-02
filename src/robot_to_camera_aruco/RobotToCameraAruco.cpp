/*
 * Calibrator.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "utils/Conversions.hpp"
#include "utils/Colors.hpp"
#include <Eigen/Dense>
#include "RobotToCameraAruco.hpp"



RobotToCameraAruco::RobotToCameraAruco(std::string node_name)
: calib_finished(true) , n(node_name){

        //    message = "Point at point 1 and press 'space' to start calibration";

        GetROSParameterValues();

        // make 3 points at the corner for calibration
        calib_points_in_board_frame.push_back(cv::Point3f(0.0, 0.0, 0.0));
        calib_points_in_board_frame.push_back(cv::Point3f(0.07, 0.0, 0.0));
        calib_points_in_board_frame.push_back(cv::Point3f(0.0, 0.04, 0.0));

};



void RobotToCameraAruco::DrawCalibrationAxis(cv::String &instructions, cv::Mat img){

        std::ostringstream oss;
        auto n_points = measured_points.size();


        // project the points for both axes
        std::vector<cv::Point3f> axisPoints {
                cv::Point3f( 0,  0,  0),
                                cv::Point3f( 1,  0,  0) * visual_axis_length,
                                cv::Point3f( 0,  1,  0) * visual_axis_length,
        };

        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(
                        axisPoints, board_to_cam_rvec, board_to_cam_tvec, camera_intrinsics.camMatrix,
                        camera_intrinsics.distCoeffs, imagePoints);


        if(n_points < num_points_per_axis){
                //we are still doing the first axis (x)
                oss << "Take " << (num_points_per_axis - n_points) << " points from the x axis axis.";
                // draw x axis
                cv::line(img, imagePoints[0], imagePoints[1], Colors::Blue, 2, CV_AA);

        }else if(n_points < 2*num_points_per_axis){
                // we are at the second axis (y)
                oss << "Take " << (2*num_points_per_axis - n_points) << " points from the y axis axis.";

                // draw y axis
                cv::line(img, imagePoints[0], imagePoints[2], Colors::Green, 2, CV_AA);
        }
        else{
                // we are done
                oss << "Received all the points.";
        }

        instructions = oss.str();
}



//-----------------------------------------------------------------------------------
// CALIB BOARD ROBOT CLASS METHODS
//-----------------------------------------------------------------------------------

void RobotToCameraAruco::DrawToolTarget(cv::String &instructions, cv::Mat img) {

        cv::projectPoints(
                        calib_points_in_board_frame, board_to_cam_rvec,
                        board_to_cam_tvec, camera_intrinsics.camMatrix,
                        camera_intrinsics.distCoeffs, calib_points_screen);

        size_t pointsGot = axis_points_Old.size();
        cv::circle(img, calib_points_screen[pointsGot], 6, cv::Scalar(255, 255, 0), 1, CV_AA);
        if (pointsGot < 3) {
                // Need more points. show the message for the next point
                std::ostringstream oss;
                oss << "Move the tool tip to point " << pointsGot + 1;
                instructions = oss.str();
        } else {
                instructions = "Calibration Done.";
        }
}


void RobotToCameraAruco::SaveCalibrationPointOld(geometry_msgs::Pose::_position_type position) {

        axis_points_Old.push_back(cv::Point3d(position.x, position.y, position.z));
        std::cout << "Added calibration point <" << axis_points_Old.size()+1 << "> : "
                        << position.x << " " << position.y << " " << position.z << std::endl;

        size_t pointsGot = axis_points_Old.size();
        if (pointsGot == 3) {
                // got all the 3 points, find the transformation
                MakeTr(axis_points_Old, calib_rotm, calib_tvec);

                // set the status as done
                calib_finished = true;
        }

}



void RobotToCameraAruco::SaveCalibrationPoint(geometry_msgs::Pose::_position_type position) {

        size_t n_points = measured_points.size();

        if (n_points < 2*num_points_per_axis){
                measured_points.push_back(Eigen::Vector3d(position.x, position.y, position.z));
                std::cout << "Added point <" << n_points + 1 << "> : "
                                << position.x << " " << position.y << " " << position.z << std::endl;
        }

        // if we got all the points find the transformation
        if (n_points == 2*num_points_per_axis) {
                ROS_INFO("Finding the transformation...");
                // got all the points, find the transformation
                CalculateTransformationN(measured_points, calib_rotm, calib_tvec);
                // set the status as done
                calib_finished = true;
        }

}


void RobotToCameraAruco::MakeTr(
                std::vector<cv::Point3d> axis_points,
                cv::Matx33d &_rotm, cv::Vec3d &br_tvec) {

        br_tvec = axis_points[0];

        cv::Vec3d x = cv::Vec3d(
                        axis_points[1].x - axis_points[0].x,
                        axis_points[1].y - axis_points[0].y,
                        axis_points[1].z - axis_points[0].z);

        cv::Vec3d y = cv::Vec3d(
                        axis_points[2].x - axis_points[0].x,
                        axis_points[2].y - axis_points[0].y,
                        axis_points[2].z - axis_points[0].z);

        cv::normalize(x, x);
        cv::normalize(y, y);
        cv::Vec3d z = x.cross(y);
        cv::normalize(z, z);
        x = y.cross(z);
        y = z.cross(x);
        z = x.cross(y);
        // just to be sure!
        cv::normalize(x, x);
        cv::normalize(y, y);
        cv::normalize(z, z);

        cv::Matx33d br_rotm = cv::Matx33d::ones();

        _rotm(0, 0) = x[0];
        _rotm(0, 1) = y[0];
        _rotm(0, 2) = z[0];
        _rotm(1, 0) = x[1];
        _rotm(1, 1) = y[1];
        _rotm(1, 2) = z[1];
        _rotm(2, 0) = x[2];
        _rotm(2, 1) = y[2];
        _rotm(2, 2) = z[2];

        std::cout << "Rotation:\n"
                        << _rotm(0, 0) << " " << _rotm(0, 1) << " " << _rotm(0, 2) << "\n"
                        << _rotm(1, 0) << " " << _rotm(1, 1) << " " << _rotm(1, 2) << "\n"
                        << _rotm(2, 0) << " " << _rotm(2, 1) << " " << _rotm(2, 2) << std::endl;
        std::cout << "Translation:\n"
                        << br_tvec[0] << "\n" << br_tvec[1] << "\n" << br_tvec[2] << std::endl;
}





void RobotToCameraAruco::CalculateTransformation(
                std::vector <cv::Point3d> axis_points ,
                cv::Matx33d &rotm, cv::Vec3d &br_tvec)
{

        int num_sample=axis_points.size();
        int num_points=(num_sample-1)/2;

        Eigen::MatrixXd points;
        for (unsigned int i=0; i<num_points; i++){
                points(i,0)=axis_points[i].x;
                points(i,1)=axis_points[i].y;
                points(i,2)=axis_points[i].z;
        }

        Eigen::Vector3d points_mean;
        points_mean=points.colwise().mean();

        Eigen::MatrixXd points_detrended(1+2*num_points,3);


        for (unsigned int i=0; i<3; i++){
                for (unsigned int j=0; j<9; j++){
                        points_detrended(j,i) = points(j,i) - points_mean(i);
                }
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(points_detrended, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::Vector3d n;
        Eigen::MatrixXd V;

        V=svd.matrixV();

        for (unsigned int i=0; i<3; i++){
                n(i)=V(i,2);
        }

        Eigen::Vector3d v_vec, distance;
        double dist;
        Eigen::MatrixXd  points_proj(1 + 2*num_points, 3);


        for (unsigned int iter=0; iter<num_points*2+1; iter++){
                for(unsigned int i=0; i<3; i++){
                        v_vec(i)=points(iter,i)-points_mean(i);
                }
                dist=v_vec(0)*n(0)+v_vec(1)*n(1)+v_vec(2)*n(2);
                for(unsigned int i=0; i<3; i++){
                        distance=dist*(n.transpose());
                        points_proj(iter,i) = points(iter,i)-distance(i);
                }

        }


        // First axis

        Eigen::MatrixXd x1(num_points,1), y1(num_points,1) ,z1(num_points,1);

        for (unsigned int i=1;i<1+num_points; i++){
                x1(i-1)=points_proj(i,0);
                y1(i-1)=points_proj(i,1);
                z1(i-1)=points_proj(i,2);}

        Eigen::Vector3d P1;
        P1(0)=x1.mean();
        P1(1)=y1.mean();
        P1(2)=z1.mean();

        Eigen::Matrix3d P1_detrended;
        for (unsigned int i=0;i<3; i++){
                P1_detrended(i,0)=x1(i)-P1(0);
                P1_detrended(i,1)=y1(i)-P1(1);
                P1_detrended(i,2)=z1(i)-P1(2);
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd1(P1_detrended, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd V1;
        V1=svd1.matrixV();
        // cout << "Here is the estimate V:\n" << V1 << endl;
        Eigen::Vector3d N1;
        for (unsigned int i=0;i<3; i++){
                N1(i)=V1(i,0)/V1(2,0);
        }

        // cout << "Here is the estimate N:\n" << N1 << endl;
        Eigen::Vector3d x_axis;
        x_axis=N1/std::sqrt((N1.dot(N1)));

        Eigen::Vector3d proj;
        for (unsigned int i=0;i<3; i++){
                proj(i)=points_proj(num_points,i);
        }

        if(x_axis.dot(proj)<0) x_axis=-x_axis;


        // Second axis

        Eigen::MatrixXd x2(num_points,1), y2(num_points,1) ,z2(num_points,1);

        for (unsigned int i=1+num_points;i<1+2*num_points; i++){
                x2(i-1-num_points)=points_proj(i,0);
                y2(i-1-num_points)=points_proj(i,1);
                z2(i-1-num_points)=points_proj(i,2);}

        Eigen::Vector3d P2;
        P2(0)=x2.mean();
        P2(1)=y2.mean();
        P2(2)=z2.mean();

        //cout << "Here is the estimate V:\n" << P2 << endl;

        Eigen::Matrix3d P2_detrended;
        for (unsigned int i=0;i<3; i++){
                P2_detrended(i,0)=x2(i)-P2(0);
                P2_detrended(i,1)=y2(i)-P2(1);
                P2_detrended(i,2)=z2(i)-P2(2);
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(P2_detrended, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd V2;
        V2=svd2.matrixV();

        // cout << "Here is the estimate V:\n" << V1 << endl;

        Eigen::Vector3d N2;
        for (unsigned int i=0;i<3; i++){
                N2(i)=V2(i,0)/V2(2,0);
        }

        // cout << "Here is the estimate N:\n" << N1 << endl;
        Eigen::Vector3d y_axis;
        y_axis=N2/std::sqrt((N2.dot(N2)));

        for (unsigned int i=0;i<3; i++){
                proj(i)=points_proj(num_points,i);
        }

        if(y_axis.dot(proj)<0) y_axis=-y_axis;

        //cout << "Here is the estimate y_axis:\n" << y_axis << endl;


        // Third axis

        Eigen::Vector3d z_axis;
        z_axis=x_axis.cross(y_axis);
        z_axis=z_axis/std::sqrt((z_axis.dot(z_axis)));

        // cout << "Here is the estimate z_axis:\n" << z_axis << endl;


        // Assure orthonormality

        y_axis=z_axis.cross(x_axis);

        // Interception

        Eigen::Vector3d ausiliar_vec;

        ausiliar_vec(0)=x1(0)-P1(0);
        ausiliar_vec(1)=y1(0)-P1(1);
        ausiliar_vec(2)=z1(0)-P1(2);
        Eigen::Vector3d A1;
        A1=P1 + N1.dot(ausiliar_vec) * N1/(N1.dot(N1));
        //cout << "Here is the estimate ausiliar vec:\n" << ausiliar_vec << endl;
        //cout << "Here is the estimate A1:\n" << A1 << endl;

        ausiliar_vec(0)=x1(3)-P1(0);
        ausiliar_vec(1)=y1(3)-P1(1);
        ausiliar_vec(2)=z1(3)-P1(2);
        Eigen::Vector3d B1;
        B1=P1+N1.dot(ausiliar_vec)*N1/(N1.dot(N1));
        ausiliar_vec(0)=x2(0)-P2(0);
        ausiliar_vec(1)=y2(0)-P2(1);
        ausiliar_vec(2)=z2(0)-P2(2);
        Eigen::Vector3d A2;
        A2=P2+N2.dot(ausiliar_vec)*N2/(N2.dot(N2));

        ausiliar_vec(0)=x2(3)-P2(0);
        ausiliar_vec(1)=y2(3)-P2(1);
        ausiliar_vec(2)=z2(3)-P2(2);
        Eigen::Vector3d B2;
        B2=P2+N2.dot(ausiliar_vec)*N2/(N2.dot(N2));

        Eigen::Vector3d L,M,N;
        L<<A1-A2;
        M<<A1-B1;
        N<<A2-B2;
        //cout << "Here is the estimate A1:\n" << A1 << endl;
        //cout << "Here is the estimate A2:\n" << A2 << endl;
        //cout << "Here is the estimate L:\n" << L << endl;

        Eigen::Matrix <double, 3,2> A;
        A(0,0)=M(0);
        A(1,0)=M(1);
        A(2,0)=M(2);
        A(0,1)=N(0);
        A(1,1)=N(1);
        A(2,1)=N(2);

        //cout << "Here is the estimate M:\n" << M << endl;
        //cout << "Here is the estimate N:\n" << N << endl;
        //cout << "Here is the estimate A=M|N:\n" << A << endl;

        Eigen::Vector2d T;
        Eigen::Matrix2d A_;
        A_=A.transpose()*A;
        T=(A_.inverse())*A.transpose()*L;
        //cout << "Here is the estimate T:\n" << T << endl;

        Eigen::Vector3d origin;
        origin<<A1-T(0)*(A1-B1);

        rotm(0,0)=x_axis(0);
        rotm(1,0)=x_axis(1);
        rotm(2,0)=x_axis(2);
        rotm(0,1)=y_axis(0);
        rotm(1,1)=y_axis(1);
        rotm(2,1)=y_axis(2);
        rotm(0,2)=z_axis(0);
        rotm(1,2)=z_axis(1);
        rotm(2,2)=z_axis(2);

        br_tvec(0)=origin(0);
        br_tvec(1)=origin(1);
        br_tvec(2)=origin(2);



}



std::pair<Eigen::Vector3d, Eigen::Vector3d>
RobotToCameraAruco::FindAxis(const Eigen::MatrixXd &axis_points){

        auto n_points = axis_points.rows();

        // convert to std vector
        std::vector<Eigen::Vector3d> points;
        for (size_t i = 0; i < n_points; ++i)
                points.push_back(axis_points.row(i));

        // fit the points and normalize
        auto axis_x_pair = FitLineToPoints(points);

        Eigen::Vector3d x_axis = axis_x_pair.second;
        x_axis.normalize();

        // finding the correct direction of the axis
        if(x_axis.dot(axis_points.row(0)) < 0) x_axis = -x_axis;

        return std::make_pair(axis_x_pair.first, x_axis);

}





void RobotToCameraAruco::CalculateTransformationN(
                std::vector <Eigen::Vector3d> axis_points ,
                cv::Matx33d &rotm, cv::Vec3d &br_tvec)
{

        int n_points = axis_points.size();
        int n_points_per_axis = (n_points)/2;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> points(n_points, 3);
        for (uint i=0; i<n_points; i++)	points.row(i) = axis_points[i];

        // ---------------  fit plane to the points
        auto origin_axis_pair = FitPlaneToPoints(axis_points);
        Eigen::Vector3d plane_origin = origin_axis_pair.first;
        Eigen::Vector3d plane_normal = origin_axis_pair.second;

        // --------------- project the points on the plane
        Eigen::MatrixXd centered = points.rowwise() - plane_origin.transpose();

        // the dot product of the vector from each point to center and the plane normal
        auto dist = centered * plane_normal;

        Eigen::MatrixXd projection_vectors(n_points, 3);

        for (uint iter=0; iter<n_points; iter++)
                projection_vectors.row(iter) = dist(iter) * plane_normal;

        Eigen::MatrixXd points_projected = points - projection_vectors;

        // ---------------  fit the x axis
        auto x_axis_pair = FindAxis(points_projected.topRows(n_points_per_axis));
        Eigen::Vector3d x_axis_point = x_axis_pair.first;
        Eigen::Vector3d x_axis = x_axis_pair.second;

        //  --------------- fit the y axis
        auto y_axis_pair = FindAxis(points_projected.bottomRows(n_points_per_axis));
        Eigen::Vector3d y_axis_point = y_axis_pair.first;
        Eigen::Vector3d y_axis = y_axis_pair.second;

        //  --------------- find the z axis
        Eigen::Vector3d z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // make sure y is normal to xz plane
        y_axis=z_axis.cross(x_axis);


        // Find the intersection of x and y
        // solving as a least-square problem Rp=q
        // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R = (I - x_axis*x_axis.transpose())
        				+ (I - y_axis*y_axis.transpose());

        Eigen::VectorXd q = (I - x_axis*x_axis.transpose()) * x_axis_point
        				+ (I - y_axis*y_axis.transpose()) * y_axis_point;

        Eigen::VectorXd origin = R.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(q);

        // convert to opencv matrix and vector
        for (uint i = 0; i<3 ; i++){
        	 rotm(i,0)=x_axis(i);
        	 rotm(i,1)=y_axis(i);
        	 rotm(i,2)=z_axis(i);
        	 br_tvec(i) = origin(i);
        }


}



void RobotToCameraAruco::GetTr() {

        std::vector<double> board_to_psm_vec7(7, 0.0);
        geometry_msgs::Pose board_to_psm_pose_msg;

        conversions::matx33dToKdlRot(calib_rotm, board_to_robot_frame.M);
        board_to_robot_frame.p.data[0] = calib_tvec.val[0];
        board_to_robot_frame.p.data[1] = calib_tvec.val[1];
        board_to_robot_frame.p.data[2] = calib_tvec.val[2];

        tf::poseKDLToMsg(board_to_robot_frame, board_to_psm_pose_msg);
        conversions::poseMsgToVector(board_to_psm_pose_msg, board_to_psm_vec7);
        n.setParam("board_to_robot_tr", board_to_psm_vec7);
        std::cout << "The board_to_robot_tr parameter was set to: " << board_to_psm_vec7
                        << std::endl;

        // print the pose of board_to_cam
        geometry_msgs::Pose temp_pose;
        tf::poseKDLToMsg(board_to_cam_frame.Inverse(), temp_pose);
        std::cout << "  -> Cam to board: " << temp_pose << std::endl;

        tf::poseKDLToMsg(board_to_robot_frame.Inverse(), temp_pose);
        std::cout << "  -> RCM of tool to board: " << temp_pose << std::endl;
}


void RobotToCameraAruco::Reset() {
        //    std::cout << "Initial size of axis points = " << axisPointsOld.size() << std::endl;
        calib_finished = false;

        axis_points_Old.clear();

}




//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void RobotToCameraAruco::GetROSParameterValues() {
        bool all_required_params_found = true;

        n.param<double>("frequency", ros_freq, 25);

        if (n.getParam("visual_axis_length", visual_axis_length))
                ROS_INFO_STREAM("Using parameter " << n.resolveName("visual_axis_length").c_str() <<
                                " with value " << visual_axis_length);


        if (!n.getParam("number_of_points_per_axis", num_points_per_axis))
                ROS_INFO_STREAM("Parameter" << n.resolveName("number_of_points_per_axis").c_str() <<
                                ". Default value is used: " << num_points_per_axis);
        else
                ROS_INFO_STREAM("Using parameter "
                                <<n.resolveName("number_of_points_per_axis").c_str()
                                << "with value " << num_points_per_axis << " points per axis. ");


        // load the intrinsic calibration file
        std::string cam_intrinsic_calibration_file_path;
        if (n.getParam("cam_intrinsic_calibration_file_path",
                        cam_intrinsic_calibration_file_path)) {
                ReadCameraParameters(cam_intrinsic_calibration_file_path);
        } else {
                ROS_ERROR("Parameter '%s' is required.",
                                n.resolveName("cam_intrinsic_calibration_file_path").c_str());
        }


        // a topic name is required for the images
        std::string cam_image_topic_name;
        if (n.getParam("camera_image_topic_name", cam_image_topic_name)) {

                // if the topic name is found, check if something is being published on it
                if (!ros::topic::waitForMessage<sensor_msgs::Image>(
                                cam_image_topic_name, ros::Duration(1))) {
                        ROS_WARN("Topic '%s' is not publishing.", cam_image_topic_name.c_str());
//			all_required_params_found = false;
                }
                else
                        ROS_INFO("Reading camera images from topic '%s'", cam_image_topic_name.c_str());

        } else {
                ROS_ERROR("Parameter '%s' is required.",
                                n.resolveName("camera_image_topic_name").c_str());
                all_required_params_found = false;
        }

        // register image subscriber
        camera_image_subscriber = n.subscribe(cam_image_topic_name, 10,
                        &RobotToCameraAruco::CameraImageCallback, this);



        // a topic name is required for the camera pose
        std::string cam_pose_topic_name;
        if (n.getParam("cam_pose_topic_name", cam_pose_topic_name)) {

                // if the topic name is found, check if something is being published on it
                if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>( cam_pose_topic_name,
                                ros::Duration(1))) {
                        ROS_WARN("Topic '%s' is not publishing.", cam_pose_topic_name.c_str());
//			all_required_params_found = false;
                }
                else
                        ROS_INFO("Reading camera pose from topic '%s'", cam_pose_topic_name.c_str());
        } else {
                ROS_ERROR("Parameter '%s' is required.",
                                n.resolveName("cam_pose_topic_name").c_str());
                all_required_params_found = false;
        }

        // register camera pose subscriber
        camera_pose_subscriber = n.subscribe(cam_pose_topic_name, 10,
                        &RobotToCameraAruco::CameraPoseCallback, this);


        // a topic name is required for the robot pose
        std::string robot_pose_topic_name;
        if (n.getParam("robot_pose_topic_name", robot_pose_topic_name)) {

                // if the topic name is found, check if something is being published on it
                if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>( robot_pose_topic_name, ros::Duration(1))) {
                        ROS_ERROR("Topic '%s' is not publishing.", robot_pose_topic_name.c_str());
                        //    		all_required_params_found = false;
                }
                else
                        ROS_INFO("Reading robot pose from topic '%s'", robot_pose_topic_name.c_str());
        } else {
                ROS_ERROR("Parameter '%s' is required.", n.resolveName("robot_pose_topic_name").c_str());
                all_required_params_found = false;
        }

        // register camera pose subscriber
        robot_pose_subscriber = n.subscribe(robot_pose_topic_name, 10, &RobotToCameraAruco::RobotPoseCallback, this);

        if (!all_required_params_found)
                throw std::runtime_error("ERROR: some required topics are not set");

}


void RobotToCameraAruco::ReadCameraParameters(std::string file_path) {
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        ROS_INFO_STREAM("Reading camera intrinsic data from: " << file_path);

        if (!fs.isOpened())
                throw std::runtime_error("Unable to read the camera parameters file.");

        fs["camera_matrix"] >> camera_intrinsics.camMatrix;
        fs["distortion_coefficients"] >> camera_intrinsics.distCoeffs;

        // check if we got something
        if(camera_intrinsics.distCoeffs.empty()){
                ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
                throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
        }
        if(camera_intrinsics.camMatrix.empty()){
                ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
                throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
        }
}



void RobotToCameraAruco::CameraImageCallback(const sensor_msgs::ImageConstPtr &msg) {
        try
        {
                image_msg = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}


void RobotToCameraAruco::CameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

        //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
        camera_pose.pose.position = msg->pose.position;
        camera_pose.pose.orientation = msg->pose.orientation;

        // converting to frame and rvec/tvec
        tf::poseMsgToKDL(camera_pose.pose, board_to_cam_frame);
        conversions::kdlFrameToRvectvec(board_to_cam_frame,board_to_cam_rvec, board_to_cam_tvec );

}

cv::Mat& RobotToCameraAruco::Image(ros::Duration timeout) {
        ros::Rate loop_rate(0.2);
        ros::Time timeout_time = ros::Time::now() + timeout;

        while(image_msg.empty()) {
                ros::spinOnce();
                loop_rate.sleep();

                if (ros::Time::now() > timeout_time) {
                        ROS_WARN("Timeout whilst waiting for a new image from the image topic. "
                                        "Is the camera publishing?");
                }
        }

        return image_msg;
}

void RobotToCameraAruco::RobotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
        robot_pose.pose.position = msg->pose.position;
        robot_pose.pose.orientation = msg->pose.orientation;

}

std::ostream &operator<<(std::ostream &out, const std::vector<double> &vect) {
        for (unsigned int iter = 0; iter < vect.size(); ++iter) {
                out << "[" << iter << "]: " << vect.at(iter) << "\t";
        }

        return out;
}



