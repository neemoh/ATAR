//
// Created by charm on 2/21/17.
//

#include "OverlayGraphics.h"
#include <utils/Conversions.hpp>
#include "opencv2/calib3d/calib3d.hpp"

OverlayGraphics::OverlayGraphics(std::string node_name, int width, int height)
: n(node_name), image_width_(width), image_height_(height),
  both_cam_poses_are_published(false)
{

    bufferL_ = new unsigned char[image_width_*image_height_*4];
    bufferR_ = new unsigned char[image_width_*image_height_*4];
    it = new image_transport::ImageTransport(n);
    GetROSParameterValues();


}



void OverlayGraphics::ReadCameraParameters(std::string file_path) {
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    ROS_INFO("Reading camera intrinsic data from: '%s'",file_path.c_str());

    if (!fs.isOpened())
        throw std::runtime_error("Unable to read the camera parameters file.");

    fs["camera_matrix"] >> Camera.camMatrix;
    fs["distortion_coefficients"] >> Camera.distCoeffs;

    // check if we got osomething
    if(Camera.distCoeffs.empty()){
        ROS_ERROR("distortion_coefficients was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }
    if(Camera.camMatrix.empty()){
        ROS_ERROR("camera_matrix was not found in '%s' ", file_path.c_str());
        throw std::runtime_error("ERROR: Intrinsic camera parameters not found.");
    }


}


void OverlayGraphics::DrawCube(cv::InputOutputArray image, const CameraDistortion &cam_intrinsics,
                               const cv::Vec3d &rvec, const cv::Vec3d &tvec, const cv::Point3d &origin,
                               const cv::Point3d &whd, const cv::Scalar & color){

    CV_Assert(image.getMat().total() != 0 &&
              (image.getMat().channels() == 1 || image.getMat().channels() == 3));

    // project axis points
    std::vector<cv::Point3f> axisPoints;

    axisPoints.push_back(cv::Point3d(origin.x, origin.y + whd.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y + whd.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y, origin.z));
    axisPoints.push_back(cv::Point3d(origin.x, origin.y, origin.z));

    axisPoints.push_back(cv::Point3d(origin.x, origin.y + whd.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y + whd.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x + whd.x, origin.y, origin.z + whd.z));
    axisPoints.push_back(cv::Point3d(origin.x, origin.y, origin.z + whd.z));

    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec,
                      cam_intrinsics.camMatrix, cam_intrinsics.distCoeffs, imagePoints);

    // draw axis lines
    int points[7] = {0, 1, 2, 4, 5, 6};
    for (int i = 0; i < 6; i++) {
        line(image, imagePoints[points[i]], imagePoints[points[i] + 1],
             color, 2, CV_AA);
    }
    for (int i = 0; i < 4; i++) {
        line(image, imagePoints[i], imagePoints[i + 4], color, 2, CV_AA);
    }
    line(image, imagePoints[3], imagePoints[0], color, 2, CV_AA);
    line(image, imagePoints[7], imagePoints[4], color, 2, CV_AA);

}


//-----------------------------------------------------------------------------------
// GetROSParameterValues
//-----------------------------------------------------------------------------------

void OverlayGraphics::GetROSParameterValues() {
    bool all_required_params_found = true;

    n.param<double>("frequency", ros_freq, 80);
    ROS_INFO_STREAM("Running at " << ros_freq);

    // load the intrinsic calibration file
    std::string cam_intrinsic_calibration_file_path;
    if (n.getParam("cam_intrinsic_calibration_file_path", cam_intrinsic_calibration_file_path)) {
        ReadCameraParameters(cam_intrinsic_calibration_file_path);
    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName(
                "cam_intrinsic_calibration_file_path").c_str());
    }


    //--------
    // Left image subscriber
    std::string left_image_topic_name = "/camera/left/image_color";;
    if (n.getParam("left_image_topic_name", left_image_topic_name))
            ROS_INFO("For left camera images will subscribe to topic '%s'", left_image_topic_name.c_str());

    image_subscriber_left = it->subscribe(
            left_image_topic_name, 1, &OverlayGraphics::ImageLeftCallback, this);

    //--------
    // Left image subscriber. Get the topic name parameter and make sure it is being published
    std::string right_image_topic_name = "/camera/right/image_color";
    if (n.getParam("right_image_topic_name", right_image_topic_name))
        ROS_INFO("[SUBSCRIBERS] Right camera images be read from topic '%s'", right_image_topic_name.c_str());

    image_subscriber_right = it->subscribe(
            right_image_topic_name, 1, &OverlayGraphics::ImageRightCallback, this);


    // the transformation from left cam coordinate frame to right cam. If this is not
    // available then both cam poses  must be publishing.
    std::vector<double> left_to_right_cam_transform = std::vector<double>(7, 0.0);
    if(n.getParam("left_cam_to_right_cam_transform", left_to_right_cam_transform))
        conversions::VectorToKDLFrame(left_to_right_cam_transform, left_cam_to_right_cam_tr);
    else
        both_cam_poses_are_published = true;

    //--------
    // We need to know the pose of the camera with respect to the task-space coordinates.
    // If the camera or the markers move the pose should be estimated by a node and here we
    // subscribe to that topic. If on the other hand no cam/marker motion is involved the
    // fixed pose is read as a static parameter here.

    // If a parameter is set, use that
    // Left image pose subscriber. Get the topic name parameter and make sure it is being published
    std::string left_cam_pose_topic_name;
    if (n.getParam("left_cam_pose_topic_name", left_cam_pose_topic_name)) {
        // if the topic name is found, check if something is being published on it
        if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                left_cam_pose_topic_name, ros::Duration(2)))
            ROS_WARN("Topic '%s' is not publishing.", left_cam_pose_topic_name.c_str());
        else
            ROS_INFO("[SUBSCRIBERS] Left camera pose will be read from topic '%s'", left_cam_pose_topic_name.c_str());
    } else {
        ROS_ERROR("Parameter '%s' is required.", n.resolveName("left_cam_pose_topic_name").c_str());
        all_required_params_found = false;
    }
    camera_pose_subscriber_left = n.subscribe(
            left_cam_pose_topic_name, 1, &OverlayGraphics::LeftCamPoseCallback, this);

    // Right camera pose will be needed only if the fixed transformation between right and
    // left camera is not provided as a parameter.
    if(both_cam_poses_are_published){

        // Right image pose subscriber. Get the topic name parameter and make sure it is being published
        std::string right_cam_pose_topic_name;
        if (n.getParam("right_cam_pose_topic_name", right_cam_pose_topic_name)) {
            // if the topic name is found, check if something is being published on it
            if (!ros::topic::waitForMessage<geometry_msgs::PoseStamped>(
                    right_cam_pose_topic_name, ros::Duration(2)))
                ROS_WARN("left_cam_to_right_cam_transform parameter was not provided and "
                                 "topic '%s' is not being published yet.", right_cam_pose_topic_name.c_str());
            else
                ROS_INFO("[SUBSCRIBERS] Right camera pose will be read from topic '%s'", right_cam_pose_topic_name.c_str());
        } else {
            ROS_ERROR("Since left_cam_to_right_cam_transform parameter was not provided"
                              "parameter '%s' is required.", n.resolveName("right_cam_pose_topic_name").c_str());
            all_required_params_found = false;
        }
        camera_pose_subscriber_right = n.subscribe(
                right_cam_pose_topic_name, 1, &OverlayGraphics::RightCamPoseCallback, this);
    }

    //----------
    // to find the transformation between the cameras I defined a simple service.
    stereo_tr_calc_client =
            n.serviceClient<teleop_vision::CalculateStereoCamsTransfromFromTopics>
                    ("/calculate_stereo_cams_transform_from_topics");
    std::string right_cam_pose_topic_name;
    n.getParam("right_cam_pose_topic_name", right_cam_pose_topic_name);

    stereo_tr_srv.request.cam_1_pose_topic_name = left_cam_pose_topic_name;
    stereo_tr_srv.request.cam_2_pose_topic_name = right_cam_pose_topic_name;


    if (!all_required_params_found)
        throw std::runtime_error("ERROR: some required topics are not set");


    //--------
    // PSM1 pose subscriber
    std::vector<double> taskspace_to_psm1_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("taskspace_to_psm1_tr", taskspace_to_psm1_vec))
        ROS_WARN("Parameter %s is not set. This parameter is required"
                         "if PSM2 is used is required.",
                 n.resolveName("taskspace_to_psm1_tr").c_str());
    conversions::VectorToKDLFrame(taskspace_to_psm1_vec, taskspace_to_psm1_tr);

    //--------
    // PSM! pose subscriber
    std::vector<double> taskspace_to_psm2_vec = std::vector<double>(7, 0.0);
    if(!n.getParam("taskspace_to_psm2_tr", taskspace_to_psm2_vec))
        ROS_WARN("Parameter %s is not set. This parameter is required"
                         "if PSM2 is used is required.",
                 n.resolveName("taskspace_to_psm2_tr").c_str());

    conversions::VectorToKDLFrame(taskspace_to_psm2_vec, taskspace_to_psm2_tr);

    //--------
    // PSM1 pose subscriber.
    std::string psm1_pose_topic_name = "/dvrk/PSM1/position_cartesian_current";
    psm1_pose_sub =
            n.subscribe(psm1_pose_topic_name, 2,
                        &OverlayGraphics::PSM1PoseCallback, this);
    ROS_INFO("Subscribing to '%s' ", psm1_pose_topic_name.c_str());

    //--------
    // PSM2 pose subscriber.
    std::string psm2_pose_topic_name = "/dvrk/PSM2/position_cartesian_current";
    psm2_pose_sub =
            n.subscribe(psm2_pose_topic_name, 2,
                        &OverlayGraphics::PSM2PoseCallback, this);
    ROS_INFO("Subscribing to %s ", psm2_pose_topic_name.c_str());



    //--------
    // PSM2 pose subscriber.
    ac_path_subscriber = n.subscribe("/ac_path", 1, &OverlayGraphics::ACPathCallback, this);

    // advertise publishers
//    std::string board_to_cam_pose_topic_name;
//    if (!n.getParam("board_to_cam_pose_topic_name", board_to_cam_pose_topic_name))
//        board_to_cam_pose_topic_name = "board_to_camera";
//
//    pub_board_to_cam_pose = n.advertise<geometry_msgs::PoseStamped>(board_to_cam_pose_topic_name, 1, 0);
//    ROS_INFO("Publishing board to camera pose on '%s'", n.resolveName(board_to_cam_pose_topic_name).c_str());

}



void OverlayGraphics::ImageRightCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_right_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void OverlayGraphics::ImageLeftCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        image_left_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void OverlayGraphics::LeftCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam_l);
    conversions::KDLFrameToRvectvec(pose_cam_l, cam_rvec_l, cam_tvec_l);

    // If we don't have the transforms between the cameras
    if(!both_cam_poses_are_published) {
        pose_cam_r = left_cam_to_right_cam_tr * pose_cam_l ;
        conversions::KDLFrameToRvectvec(pose_cam_r, cam_rvec_r, cam_tvec_r);
    }
}

void OverlayGraphics::RightCamPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{

    tf::poseMsgToKDL(msg->pose, pose_cam_r);
    conversions::KDLFrameToRvectvec(pose_cam_r, cam_rvec_r, cam_tvec_r);

}

void OverlayGraphics::PSM1PoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_psm1 = taskspace_to_psm1_tr.Inverse() * frame_temp;

}

void OverlayGraphics::PSM2PoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
    // Convert message to KDL
    KDL::Frame frame_temp;
    tf::poseMsgToKDL(msg->pose, frame_temp);

    // take the robot end-effector pose to the task space coordinate frame
    pose_psm2.p = taskspace_to_psm2_tr.M.Inverse() * (frame_temp.p - taskspace_to_psm2_tr.p);
//    pose_psm2 = taskspace_to_psm2_tr.Inverse() * frame_temp;

}

void OverlayGraphics::ACPathCallback(const geometry_msgs::PoseArrayConstPtr & msg){

    for (int n_point = 0; n_point < msg->poses.size(); ++n_point) {
        ac_path.push_back(cv::Point3d(msg->poses[n_point].position.x,
                                      msg->poses[n_point].position.y,
                                      msg->poses[n_point].position.z));
    }
}




cv::Mat& OverlayGraphics::ImageLeft(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_left_.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time)
            ROS_WARN("Timeout: No new left Image.");
    }

    return image_left_;
}



cv::Mat& OverlayGraphics::ImageRight(ros::Duration timeout) {
    ros::Rate loop_rate(10);
    ros::Time timeout_time = ros::Time::now() + timeout;

    while(image_right_.empty()) {
        ros::spinOnce();
        loop_rate.sleep();

        if (ros::Time::now() > timeout_time) {
            ROS_WARN("Timeout: No new right Image.");
        }
    }

    return image_right_;
}



void OverlayGraphics::DrawToolTip(cv::InputOutputArray image,
                                  const CameraDistortion &cam_intrinsics,
                                  const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                  KDL::Vector position,
                                  const cv::Scalar color) {

    //	Point3d board_to_robot_transl = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
//    cv::Point3d toolPoint3d_rrf = cv::Point3d(position.x, position.y, position.z);
//
//    // taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
//    cv::Point3d toolPoint3d_crf = board_to_robot_rotm.t() * (toolPoint3d_rrf - board_to_robot_transl);
//    cv::Point3d toolPoint3d_crf = cv::Point3d(temp.x , temp.y , temp.z);

    std::vector<cv::Point3d> toolPoint3d_vec_crf;
    std::vector<cv::Point2d> toolPoint2d;
    toolPoint3d_vec_crf.push_back(cv::Point3d(position[0] , position[1] , position[2]));

    projectPoints(toolPoint3d_vec_crf, rvec, tvec, cam_intrinsics.camMatrix,
                  cam_intrinsics.distCoeffs, toolPoint2d);
    circle(image, toolPoint2d[0], 20, color, -1);

}


void OverlayGraphics::DrawACPath(cv::InputOutputArray image,
                                 const CameraDistortion &cam_intrinsics,
                                 const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                 const cv::Scalar color){

    std::vector<cv::Point2d> ac_points_2d;
    projectPoints(ac_path, rvec, tvec, cam_intrinsics.camMatrix,
                  cam_intrinsics.distCoeffs, ac_points_2d);
    for (int i = 0; i < ac_path.size(); ++i) {
        circle(image, ac_points_2d[i], 2, color, -1);
    }
    
}



//
//void OverlayGraphics::drawBoundinBox(
//        float x_min, float x_max,
//        float y_min, float y_max,
//        float z_min, float z_max)
//{
//    glColor4f(0.0f, 0.8f, 0.0f, 0.5f);
//
//    glBegin(GL_QUADS);
//    // top
//    glNormal3f(0.0f, 1.0f, 0.0f);
//    glVertex3f(x_min, y_max, z_max);
//    glVertex3f(x_max, y_max, z_max);
//    glVertex3f(x_max, y_max, z_min);
//    glVertex3f(x_min, y_max, z_min);
//    glEnd();
//
//    glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
//    glBegin(GL_QUADS);
//    // front
//    glNormal3f(0.0f, 0.0f, 1.0f);
//    glVertex3f(x_max, y_min, z_max);
//    glVertex3f(x_max, y_max, z_max);
//    glVertex3f(x_min, y_max, z_max);
//    glVertex3f(x_min, y_min, z_max);
//    glEnd();
//
//    glColor4f(0.0f, 0.8f, 0.0f, 0.5f);
//    glBegin(GL_QUADS);
//    // right
//    glNormal3f(1.0f, 0.0f, 0.0f);
//    glVertex3f(x_max, y_max, z_min);
//    glVertex3f(x_max, y_max, z_max);
//    glVertex3f(x_max, y_min, z_max);
//    glVertex3f(x_max, y_min, z_min);
//
//    glEnd();
//
//    glBegin(GL_QUADS);
//    // left
//    glNormal3f(-1.0f, 0.0f, 0.0f);
//    glVertex3f(x_min, y_min, z_max);
//    glVertex3f(x_min, y_max, z_max);
//    glVertex3f(x_min, y_max, z_min);
//    glVertex3f(x_min, y_min, z_min);
//
//    glEnd();
//
//    glBegin(GL_QUADS);
//    // bottom
//    glNormal3f(0.0f, -1.0f, 0.0f);
//    glVertex3f(x_max, y_min, z_max);
//    glVertex3f(x_min, y_min, z_max);
//    glVertex3f(x_min, y_min, z_min);
//    glVertex3f(x_max, y_min, z_min);
//
//    glEnd();
//
//    glBegin(GL_QUADS);
//    // back
//    glNormal3f(0.0f, 0.0f, -1.0f);
//    glVertex3f(x_max, y_max, z_min);
//    glVertex3f(x_max, y_min, z_min);
//    glVertex3f(x_min, y_min, z_min);
//    glVertex3f(x_min, y_max, z_min);
//
//    glEnd();
//}



//
//void OverlayGraphics::drawElipsoid(
//        float x_min, float x_max,
//        float y_min, float y_max,
//        float z_min, float z_max)
//{
//    static bool firstTime = true;
//    static const int Ns = 40;
//    static const int Np = 20;
//    static float vertices[(Ns + 1) * (Np * 2 - 1) * 3];
//
//    float *vertex;
//    float x, y, z;
//    if (firstTime)
//    {
//        firstTime = false;
//
//        for (int j = 0; j < Np * 2 - 1; j++)
//        {
//            aux = (float)(j - Np + 1) / (float)Np * (float)M_PI * 0.5f;
//            z = sinf(aux);
//            cosbeta = cosf(aux);
//
//            for (int i = 0; i < Ns; i++)
//            {
//                aux = (float)i * 2.0f * (float)M_PI / (float)Ns;
//                x = cosbeta * cosf(aux);
//                y = -cosbeta * sinf(aux);
//                vertex = &vertices[(j * (Ns + 1) + i) * 3];
//                vertex[0] = x;
//                vertex[1] = y;
//                vertex[2] = z;
//            }
//
//            vertex = &vertices[(j * (Ns + 1) + Ns) * 3];
//            vertex[0] = cosbeta;
//            vertex[1] = 0.0f;
//            vertex[2] = z;
//        }
//    }
//
//    glPushMatrix();
//    glTranslatef(
//            0.5f * (x_max + x_min),
//            0.5f * (y_max + y_min),
//            0.5f * (z_max + z_min));
//    glScalef(
//            0.5f * (x_max - x_min),
//            0.5f * (y_max - y_min),
//            0.5f * (z_max - z_min));
//    for (int j = 0; j < Np * 2 - 2; j++)
//    {
//        glBegin(GL_TRIANGLE_STRIP);
//        for (int i = 0; i <= Ns; i++)
//        {
//            vertex = &vertices[(j * (Ns + 1) + i) * 3];
//            glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//            glNormal3f(vertex[0], vertex[1], vertex[2]);
//            glVertex3f(vertex[0], vertex[1], vertex[2]);
//
//            vertex = &vertices[((j + 1) * (Ns + 1) + i) * 3];
//            glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//            glNormal3f(vertex[0], vertex[1], vertex[2]);
//            glVertex3f(vertex[0], vertex[1], vertex[2]);
//        }
//        glEnd();
//    }
//
//    glBegin(GL_TRIANGLE_FAN);
//    glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
//    glNormal3f(0.0f, 0.0f, 1.0f);
//    glVertex3f(0.0f, 0.0f, 1.0f);
//    for (int i = Ns; i >= 0; i--)
//    {
//        vertex = &vertices[((Np * 2 - 2) * (Ns + 1) + i) * 3];
//        glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//        glNormal3f(vertex[0], vertex[1], vertex[2]);
//        glVertex3f(vertex[0], vertex[1], vertex[2]);
//    }
//    glEnd();
//
//    glBegin(GL_TRIANGLE_FAN);
//    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
//    glNormal3f(0.0f, 0.0f, -1.0f);
//    glVertex3f(0.0f, 0.0f, -1.0f);
//    for (int i = 0; i <= Ns; i++)
//    {
//        vertex = &vertices[i * 3];
//        glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//        glNormal3f(vertex[0], vertex[1], vertex[2]);
//        glVertex3f(vertex[0], vertex[1], vertex[2]);
//    }
//    glEnd();
//
//    glPopMatrix();
//}  float cosbeta;
//    float aux;
//
//    if (firstTime)
//    {
//        firstTime = false;
//
//        for (int j = 0; j < Np * 2 - 1; j++)
//        {
//            aux = (float)(j - Np + 1) / (float)Np * (float)M_PI * 0.5f;
//            z = sinf(aux);
//            cosbeta = cosf(aux);
//
//            for (int i = 0; i < Ns; i++)
//            {
//                aux = (float)i * 2.0f * (float)M_PI / (float)Ns;
//                x = cosbeta * cosf(aux);
//                y = -cosbeta * sinf(aux);
//                vertex = &vertices[(j * (Ns + 1) + i) * 3];
//                vertex[0] = x;
//                vertex[1] = y;
//                vertex[2] = z;
//            }
//
//            vertex = &vertices[(j * (Ns + 1) + Ns) * 3];
//            vertex[0] = cosbeta;
//            vertex[1] = 0.0f;
//            vertex[2] = z;
//        }
//    }
//
//    glPushMatrix();
//    glTranslatef(
//            0.5f * (x_max + x_min),
//            0.5f * (y_max + y_min),
//            0.5f * (z_max + z_min));
//    glScalef(
//            0.5f * (x_max - x_min),
//            0.5f * (y_max - y_min),
//            0.5f * (z_max - z_min));
//    for (int j = 0; j < Np * 2 - 2; j++)
//    {
//        glBegin(GL_TRIANGLE_STRIP);
//        for (int i = 0; i <= Ns; i++)
//        {
//            vertex = &vertices[(j * (Ns + 1) + i) * 3];
//            glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//            glNormal3f(vertex[0], vertex[1], vertex[2]);
//            glVertex3f(vertex[0], vertex[1], vertex[2]);
//
//            vertex = &vertices[((j + 1) * (Ns + 1) + i) * 3];
//            glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//            glNormal3f(vertex[0], vertex[1], vertex[2]);
//            glVertex3f(vertex[0], vertex[1], vertex[2]);
//        }
//        glEnd();
//    }
//
//    glBegin(GL_TRIANGLE_FAN);
//    glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
//    glNormal3f(0.0f, 0.0f, 1.0f);
//    glVertex3f(0.0f, 0.0f, 1.0f);
//    for (int i = Ns; i >= 0; i--)
//    {
//        vertex = &vertices[((Np * 2 - 2) * (Ns + 1) + i) * 3];
//        glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//        glNormal3f(vertex[0], vertex[1], vertex[2]);
//        glVertex3f(vertex[0], vertex[1], vertex[2]);
//    }
//    glEnd();
//
//    glBegin(GL_TRIANGLE_FAN);
//    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
//    glNormal3f(0.0f, 0.0f, -1.0f);
//    glVertex3f(0.0f, 0.0f, -1.0f);
//    for (int i = 0; i <= Ns; i++)
//    {
//        vertex = &vertices[i * 3];
//        glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
//        glNormal3f(vertex[0], vertex[1], vertex[2]);
//        glVertex3f(vertex[0], vertex[1], vertex[2]);
//    }
//    glEnd();
//
//    glPopMatrix();
//}


//void OverlayGraphics::InitGL(int w, int h)
//{
//    // Background color
//
//    glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
//    //	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
//
//    // Smooth triangles (GL_FLAT for flat triangles)
//    glShadeModel(GL_SMOOTH);
//
//    // We don't see the back face
//    glCullFace(GL_BACK);
//    // The front face is CCW
//    glFrontFace(GL_CCW);
//    // Disable culling
//    glEnable(GL_CULL_FACE);
//
//    // Disable lighting
//    glDisable(GL_LIGHTING);
//
//    // Disable depth test
//    glEnable(GL_DEPTH_TEST);
//
//    // Enable normalizing
//    glEnable(GL_NORMALIZE);
//
//    // Enable blending
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glBlendEquation(GL_FUNC_ADD);
//
//    // Generate texture
//    glGenTextures(1, &texIdL);
//    glBindTexture(GL_TEXTURE_2D, texIdL);
//    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
//
//    glGenTextures(1, &texIdR);
//    glBindTexture(GL_TEXTURE_2D, texIdR);
//    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
//
//    // Generate gauge textures
////	glGenTextures(11, gaugeTexId);
////	for (int i = 0; i < 11; i++)
////	{
////		glBindTexture(GL_TEXTURE_2D, gaugeTexId[i]);
////		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
////		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
////		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
////		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
////		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
////		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, gauge[i].cols, gauge[i].rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, gauge[i].data);
////	}
//}



//void OverlayGraphics::RenderSide(
//        GLFWwindow* window,
//        KDL::Frame &cameraPose,
//        cv::Mat &cameraMatrix,
//        unsigned char* buffer,
//        int x, int width, int height,
//        GLuint texId,
//        std::vector<cv::Point2f> safety_area)
//{
//    float distance = 0.0;
//
//    TabletInfo tablet_info;
//    bool enableImage;
//    bool enableHud;
//    // We draw in all the window
//    glViewport(x, 0, width, height);
//
//    /* Draw camera image in the background */
//
//    // Set camera
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    // Set 2D camera
//    gluOrtho2D(-1.0, 1.0, 1.0, -1.0);
//
//    glMatrixMode(GL_MODELVIEW);
//    glLoadIdentity();
//
//    // Disable
//    glDisable(GL_DEPTH_TEST);
//
//    // Draw a square with the texture
//    if(enableImage)
//    {
//        glEnable(GL_TEXTURE_2D);
//        glBindTexture(GL_TEXTURE_2D, texId);
//        glColor3f(1.0f, 1.0f, 1.0f);
//        glBegin(GL_QUADS);
//        glTexCoord2f(1.0f, 1.0f);
//        glVertex3f(1.0f, 1.0f, 0.0f);
//
//        glTexCoord2f(1.0f, 0.0f);
//        glVertex3f(1.0f, -1.0f, 0.0f);
//
//        glTexCoord2f(0.0f, 0.0f);
//        glVertex3f(-1.0f, -1.0f, 0.0f);
//
//        glTexCoord2f(0.0f, 1.0f);
//        glVertex3f(-1.0f, 1.0f, 0.0f);
//        glEnd();
//    }
//
//    /*
//    if (!enableHud)
//    {
//        glReadPixels(x, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
//        return;
//    }*/
//
//
//    if (enableHud)
//    {
//        // Draw a square with the gauge
//        int gaugeIndex = 0;
//        float dmin = 0.002;
//        float dmax = 0.02;
//
//        if (distance > dmax)
//        {
//            gaugeIndex = 0;
//        }
//        else if (distance == -1.0)
//        {
//            // Warning
//            // TODO
//        }
//        else
//        {
//            gaugeIndex = (int)(10.0f * (distance - dmax) / (dmin - dmax));
//        }
//
//        if (gaugeIndex < 0)
//            gaugeIndex = 0;
//        else if (gaugeIndex > 10)
//            gaugeIndex = 10;
//
//        glEnable(GL_TEXTURE_2D);
////		glBindTexture(GL_TEXTURE_2D, gaugeTexId[gaugeIndex]);
//        glColor3f(1.0f, 1.0f, 1.0f);
//
//        glPushMatrix();
//        if (x == 0)
//            glTranslatef(-0.2f, 0.0f, 0.0f);
//
//        glBegin(GL_QUADS);
//        glTexCoord2f(1.0f, 1.0f);
//        glVertex3f(1.0f - 0.1f, -0.734f + 0.1f, 0.0f);
//
//        glTexCoord2f(1.0f, 0.0f);
//        glVertex3f(1.0f - 0.1f, -1.0f + 0.1f, 0.0f);
//
//        glTexCoord2f(0.0f, 0.0f);
//        glVertex3f(0.5f - 0.1f, -1.0f + 0.1f, 0.0f);
//
//        glTexCoord2f(0.0f, 1.0f);
//        glVertex3f(0.5f - 0.1f, -0.734f + 0.1f, 0.0f);
//        glEnd();
//
//        glPopMatrix();
//    }
//
//
//    // Draw tablet pointer
//    if (tablet_info.buttons[0])
//    {
//        // draw the pointer
//        glDisable(GL_TEXTURE_2D);
//        glColor3f(0.5f, 1.0f, 0.5f);
//        glBegin(GL_LINES);
//        glVertex3f(tablet_info.x + 0.03f, tablet_info.y, 0.0f);
//        glVertex3f(tablet_info.x - 0.03f, tablet_info.y, 0.0f);
//        glEnd();
//        glBegin(GL_LINES);
//        glVertex3f(tablet_info.x, tablet_info.y + 0.03f, 0.0f);
//        glVertex3f(tablet_info.x, tablet_info.y - 0.03f, 0.0f);
//        glEnd();
//
//        if(safety_area.size() > 1)
//        {
//            // draw the contour
//            glDisable(GL_TEXTURE_2D);
//            glColor3f(0.5f, 1.0f, 0.5f);
//            glLineWidth(3.0);
//            glBegin(GL_LINES);
//
//            for(int i = 0 ; i < safety_area.size(); i++)
//            {
//                glVertex3f(safety_area[i].x, safety_area[i].y, 0.0f);
//            }
//
//            glEnd();
//        }
//
//
//    }
//
//
//    /* Draw 3D object */
//
//    if(distance != -1 && enableHud)
//    {
//        // Set camera
//        glMatrixMode(GL_PROJECTION);
//        glLoadIdentity();
//        // Here set the camera intrinsic parameters
//        // fov in degrees, aspect ratio, minimum distance (DO NOT PUT ZERO!), maximum distance
//
//        //	gluPerspective(45.0f, (float)width / (float)height, 0.1f, 1000.0f);
//
//        /*
//        float fy = (float)cameraMatrix.at<double>(1,1)/((float)height/2.0);
//        float fovy_rad  = 2.0*atan(1.0/fy);
//        float fovy_deg = fovy_rad * 180.0 / M_PI;
//        gluPerspective(fovy_deg, (float)width / (float)height, 0.001f, 10.0f);*/
//
//        // Calculate perspective matrix
//        GLfloat cameraPerspective[16];
//        GLfloat zmin = 0.001f;
//        GLfloat zmax = 10.0f;
//
//        /*
//        cameraPerspective[0] = 2.0f * (float)cameraMatrix.at<double>(0, 0) / (float)width;
//        cameraPerspective[1] = 0.0f;
//        cameraPerspective[2] = 0.0f;
//        cameraPerspective[3] = 0.0f;
//
//        cameraPerspective[4] = 0.0f;
//        cameraPerspective[5] = -2.0f * (float)cameraMatrix.at<double>(1, 1) / (float)height;
//        cameraPerspective[6] = 0.0f;
//        cameraPerspective[7] = 0.0f;
//
//        cameraPerspective[8] = 2.0f * (float)cameraMatrix.at<double>(0, 2) / (float)width - 1.0f;
//        cameraPerspective[9] = 1.0f - 2.0f * (float)cameraMatrix.at<double>(1, 2) / (float)height;
//        cameraPerspective[10] = -(zmax + zmin) / (zmax - zmin);
//        cameraPerspective[11] = 1.0f;
//
//        cameraPerspective[12] = 0.0f;
//        cameraPerspective[13] = 0.0f;
//        cameraPerspective[14] = 2.0f * zmax * zmin / (zmax - zmin);
//        cameraPerspective[15] = 0.0f;*/
//
//        cameraPerspective[0] = 2.0f * (float)cameraMatrix.at<double>(0, 0) / (float)width;
//        cameraPerspective[1] = 0.0f;
//        cameraPerspective[2] = 0.0f;
//        cameraPerspective[3] = 0.0f;
//
//        cameraPerspective[4] = 0.0f;
//        cameraPerspective[5] = 2.0f * (float)cameraMatrix.at<double>(1, 1) / (float)height;
//        cameraPerspective[6] = 0.0f;
//        cameraPerspective[7] = 0.0f;
//
//        cameraPerspective[8] = 2.0f * (float)cameraMatrix.at<double>(0, 2) / (float)width - 1.0f;
//        cameraPerspective[9] = 2.0f * (float)cameraMatrix.at<double>(1, 2) / (float)height - 1.0f;
//        cameraPerspective[10] = (zmax + zmin) / (zmin - zmax);
//        cameraPerspective[11] = -1.0f;
//
//        cameraPerspective[12] = 0.0f;
//        cameraPerspective[13] = 0.0f;
//        cameraPerspective[14] = 2.0f * zmax * zmin / (zmin - zmax);
//        cameraPerspective[15] = 0.0f;
//
//        glMultMatrixf(cameraPerspective);
//
//        //std::cout << "fov: " << fovy_deg << std::endl;
//        glMatrixMode(GL_MODELVIEW);
//        glLoadIdentity();
//        // Here set the camera INVERSE transform using glMultMatrixf(transform)
////        GLfloat camera_pose2[16] = {
////                -0.707107, -0.408248, 0.57735, 0.0,
////                0.707107, -0.408248, 0.57735, 0.0,
////                0.0, 0.816497, 0.57735, 0.0,
////                0.0, 0.0, -3.4641, 1.0};
//        GLfloat camera_pose[16];
//
//        KDL::Frame cv2gl;
//        cv2gl = KDL::Frame::Identity();
//        cv2gl.M(0, 0) = 1.0;
//        cv2gl.M(1, 1) = -1.0;
//        cv2gl.M(2, 2) = -1.0;
//        //KDL::Frame cameraPoseInv = (cv2gl * cameraPose ).Inverse();
//        KDL::Frame cameraPoseInv = (cameraPose * cv2gl).Inverse();
//        //KDL::Frame cameraPoseInv = cv2gl * cameraPose.Inverse();
//        //KDL::Frame cameraPoseInv = cameraPose.Inverse() * cv2gl;
//
//        camera_pose[0] = (float)cameraPoseInv.M(0,0);
//        camera_pose[1] = (float)cameraPoseInv.M(1,0);
//        camera_pose[2] = (float)cameraPoseInv.M(2,0);
//
//        camera_pose[3] = 0.0f;
//
//        camera_pose[4] = (float)cameraPoseInv.M(0,1);
//        camera_pose[5] = (float)cameraPoseInv.M(1,1);
//        camera_pose[6] = (float)cameraPoseInv.M(2,1);
//
//        camera_pose[7] = 0.0f;
//
//        camera_pose[8] = (float)cameraPoseInv.M(0,2);
//        camera_pose[9] = (float)cameraPoseInv.M(1,2);
//        camera_pose[10] = (float)cameraPoseInv.M(2,2);
//
//        camera_pose[11] = 0.0f;
//
//        camera_pose[12] = (float)cameraPoseInv.p(0);
//        camera_pose[13] = (float)cameraPoseInv.p(1);
//        camera_pose[14] = (float)cameraPoseInv.p(2);
//
//        camera_pose[15] = 1.0f;
//
//        glMultMatrixf(camera_pose);
//
//        // Enable
////        glEnable(GL_DEPTH_TEST);
////
////        // Draw a square with the texture
////        glDisable(GL_TEXTURE_2D);
//
//        float bb_coord[6];
//
//        bb_coord[0] = -1.0f;
//        bb_coord[1] = 1.0f;
//        bb_coord[2] = -1.0f;
//        bb_coord[3] = 1.0f;
//        bb_coord[4] = -1.0f;
//        bb_coord[5] = 1.0f;
//
//
//
//        drawElipsoid(
//                bb_coord[0], bb_coord[1],
//                bb_coord[2], bb_coord[3],
//                bb_coord[4], bb_coord[5]);
//
//        /*drawBoundinBox(	bb_coord[0], bb_coord[1],
//                bb_coord[2], bb_coord[3],
//                bb_coord[4], bb_coord[5]);*/
//    }
//
//    glReadPixels(x, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
//}

//
//void OverlayGraphics::Render(GLFWwindow* window, TabletInfo tablet_info,
//                             std::vector<cv::Point2f> safety_area)
//{
//    // Get window dimensions
////    int width;
////    int height;
////    glfwGetFramebufferSize(window, &width, &height);
//
//    // Clear buffers
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//    RenderSide(
//            window,
//            camera_pose_left,
//            camera_matrix_l,
//            bufferL_,
//            0,
//            image_width_ / 2,
//            image_height_,
//            texIdL,
//            safety_area);
//
//    if (!tablet_info.buttons[0])
//    {
//        RenderSide(
//                window,
//                camera_pose_right,
//                camera_matrix_l,
//                bufferR_,
//                image_width_ / 2,
//                image_width_ / 2,
//                image_height_,
//                texIdR,
//                safety_area);
//    }
//    else
//    {
//        RenderSide(
//                window,
//                camera_pose_left,
//                camera_matrix_r,
//                bufferR_,
//                image_width_ / 2,
//                image_width_ / 2,
//                image_height_,
//                texIdL,
//                safety_area);
//    }
//
//    // Swap buffers
//    glfwSwapBuffers(window);
//}
//
//
//
//
//void fromMattoImageA(
//        const cv::Mat& img_in,
//        sensor_msgs::Image* img_out)
//{
//    cv_bridge::CvImage pics;
//    pics.encoding = sensor_msgs::image_encodings::RGBA8 ;
//    pics.image = img_in;
//    pics.toImageMsg(*img_out);
//    img_out->header.stamp = ros::Time::now();
//    img_out->header.frame_id = "depthimage";
//    img_out->encoding = pics.encoding;
//    img_out->width = img_in.cols;
//    img_out->height = img_in.rows;
//}


void VisualUtils::SwitchFullScreen(const std::string window_name) {

    if (cvGetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN) ==
        CV_WINDOW_NORMAL)
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    else
        cvSetWindowProperty(window_name.c_str(), CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

}