/*
 * calib_robot_board.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: nearlab
 */


#include "teleop_ac_vision_node.hpp"

using namespace std;
using namespace cv;


int main(int argc, char *argv[]) {
    cvNamedWindow("teleop", CV_WINDOW_NORMAL);
    cvSetWindowProperty("teleop", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    std::string node_name("teleop_ac_vision");
    ros::init(argc, argv, node_name);
    rosObj r(argc, argv, node_name);
    r.init();
    //	if(!r.all_good) return 1;


    //-----------------------------------------------------------------------------------
    // ros initialization
    //-----------------------------------------------------------------------------------
    ros::Rate loop_rate(r.freq_ros);
    ros::Duration d(0.5);
    ros::Subscriber sub_tool_curr = r.n.subscribe(r.tool_curr_topic_name_param, 10,
        &rosObj::toolCurrCallback, &r);
    ros::Subscriber sub_tool_dest = r.n.subscribe(r.tool_dest_topic_name_param, 10,
        &rosObj::toolDestCallback, &r);
    ros::Subscriber sub_events = r.n.subscribe(r.events_topic_name_param, 10,
        &rosObj::eventsCallback, &r);

    ros::Publisher pub_cam_to_robot_pose = r.n.advertise<geometry_msgs::Pose>("camToRobotPose", 1,
        0);
    ros::Publisher pub_ac_point_cloud = r.n.advertise<sensor_msgs::PointCloud2>("acPointCloud", 1,
        0);
    ros::Publisher pub_board_to_robot_pose = r.n.advertise<geometry_msgs::Pose>("taskToRobotPose",
        1, 0);

    int status = 1;
    vector<Point3d> calibPoints3d;
    vector<Point2d> calibPoints2d;
    geometry_msgs::Pose br_pose_msg;
    geometry_msgs::Pose cr_pose_msg;
    Mat imageCopy;
    Vec3d bc_rvec, bc_tvec, br_tvec;
    KDL::Frame bc_frame, br_frame;
    vector<double> br_vec(7, 0.0);

    Matx33d br_rotm = Matx33d::zeros();

    //-----------------------------------------------------------------------------------
    // Set up the camera
    //-----------------------------------------------------------------------------------
    Mat camMatrix, distCoeffs;

    int waitTime = 1;
    bool readOk = boardDetector::readCameraParameters(r.cam_data_path_param, camMatrix, distCoeffs);
    if (!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }
    VideoCapture inputVideo;
    inputVideo.open(r.camId_param);

    //-----------------------------------------------------------------------------------
    // Construct the board detector object
    //-----------------------------------------------------------------------------------
    if (ros::param::has(node_name + "/board_to_robot_tr")) {
        conversions::vectorToPoseMsg(r.board_to_robot_tr_param, br_pose_msg);
        tf::poseMsgToKDL(br_pose_msg, br_frame);
        conversions::kdlRotToMatx33d(br_frame.M, br_rotm);
        cout << "Set the board_to_robot_tr parameter: " << r.board_to_robot_tr_param << endl;
        ROS_INFO("Initialization done.");
    }

    //-----------------------------------------------------------------------------------
    // Construct the board detector object
    //-----------------------------------------------------------------------------------
    boardDetector bd(
        r.markersX_param,
        r.markersY_param,
        r.markerLength_px_param,
        r.markerSeparation_px_param,
        r.dictionaryId_param,
        camMatrix,
        distCoeffs,
        3);

    //-----------------------------------------------------------------------------------
    // Construct the calibBoardRobot object
    //-----------------------------------------------------------------------------------
    calibBoardRobot cbr;
    string instructions = "";

    // make 3 points at the corner of the boards for calibration
    calibPoints3d.push_back(Point3f(0, 0, 0));
    calibPoints3d.push_back(
        Point3f(bd.markersX * bd.markerLength + (bd.markersX - 1) * bd.markerSeparation, 0, 0));
    calibPoints3d.push_back(
        Point3f(0, bd.markersY * bd.markerLength + (bd.markersY - 1) * bd.markerSeparation, 0));


    //-----------------------------------------------------------------------------------
    // Construct the drawing object
    //-----------------------------------------------------------------------------------
    drawings dr(
        camMatrix,
        distCoeffs,
        br_frame,
        r.markerLength_px_param / r.markerlength_m_param);

//	dr.setSquare(Point3d(500,350,0),  Point2d(500,100));
//	dr.createEllipse(525,350,220,150,400);


    while (ros::ok() && inputVideo.isOpened()) {

        //-----------------------------------------------------------------------------------
        // DETECT BOARD
        //-----------------------------------------------------------------------------------
        inputVideo >> bd.image;
        bd.detect(bc_rvec, bc_tvec);
        if (bd.ready)
            bd.drawAxis();
        conversions::rvectvecToKdlFrame(bc_rvec, bc_tvec, bc_frame);
        //		bd.drawDetectedMarkers();
        // draw results
        bd.image.copyTo(imageCopy);

        instructions = "press k to calibrate or p to send the AC point cloud.";
        char key = (char) waitKey(waitTime);
        switch (key) {
            case 27: // Esc

                ros::shutdown();
                break;

            case 107: // k
                status = 0;
                cbr.reset();
                break;

            case 112: // p
                dr.getACCloud(r.ac_point_cloud);
                cout << "cloud size: " << r.ac_point_cloud.width * r.ac_point_cloud.height << endl;
                cout << "vec size: " << dr.ac_points_in_robot.size() << endl;
                pub_ac_point_cloud.publish(r.ac_point_cloud);
                break;

            case 114: // r
                dr.resetUsersPath();
                break;

            case 49:
                // 1: create ellipse AC points a= 0.058m, b = 0.0398 => circumference = 0.31 m
                dr.createEllipse(540, 350, 220, 150, 400);
                break;

            case 50:
                // 1: create Sine AC points
                dr.createSine(540, 350, 470, 100, 400);
                break;

            case 51:
                // 1: create hat AC points
                dr.createHat(540, 350, 400, 150, 400);
                break;

        }


//        if(key == 27)
//        	break;
//        else if(key == 107){
//        	status = 0;
//        	cbr.reset();
//        }
//        else if(key == 112){
//        	dr.getACCloud(r.ac_point_cloud);
//        	cout << "cloud size: " << r.ac_point_cloud.width * r.ac_point_cloud.height << endl;
//        	cout << "vec size: " << dr.ac_points_in_robot.size() << endl;
//        	pub_ac_point_cloud.publish(r.ac_point_cloud);
//        }
//        else if(key == 114)
//    		dr.resetUsersPath();
//        else if ()


        //-----------------------------------------------------------------------------------
        // calibration if needed
        //-----------------------------------------------------------------------------------
        if (status == 0) {

            // project the calibration points to show it in the image
            if (bd.ready) {
                projectPoints(calibPoints3d, bc_rvec, bc_tvec, camMatrix, distCoeffs,
                    calibPoints2d);

                cbr.setVisualPoints(calibPoints2d);

                cbr.updateMe(instructions, imageCopy);
            }

            if (key == 32) {

                //space pressed save the position of the point
                cbr.setCalibpoint(r.tool_curr_pose_in_slvrf.position.x,
                    r.tool_curr_pose_in_slvrf.position.y, r.tool_curr_pose_in_slvrf.position.z);

                // check the state of calibration
                if (cbr.updateMe(instructions, imageCopy)) {

                    // calibration is done
                    status = 1;

                    // get the calibration data
                    cbr.get_tr(br_rotm, br_tvec);

                    // convert the calibration data
                    conversions::matx33dToKdlRot(br_rotm, br_frame.M);
                    br_frame.p.data[0] = br_tvec.val[0];
                    br_frame.p.data[1] = br_tvec.val[1];
                    br_frame.p.data[2] = br_tvec.val[2];

                    tf::poseKDLToMsg(br_frame, br_pose_msg);
                    conversions::poseMsgToVector(br_pose_msg, br_vec);
                    r.n.setParam(node_name + "/board_to_robot_tr", br_vec);
                    cout << "Set the board_to_robot_tr parameter: " << br_vec << endl;
                }
            }

        } else {

            //-----------------------------------------------------------------------------------
            // Drawing stuff
            //-----------------------------------------------------------------------------------
            if (bd.ready) {
                dr.update_cam_2_board_ref(bc_rvec, bc_tvec);

                //        		dr.drawSquare(imageCopy);
                dr.drawACPoints(imageCopy, dr.isToolInTouch(-0.001));

                dr.transfromToBoard(r.tool_curr_pose_in_slvrf, r.tool_dest_pose_in_slvrf);
                // draw the tool tip point
//        		dr.drawToolTip(imageCopy,r.tool_curr_pose_in_slvrf.position.x , r.tool_curr_pose_in_slvrf.position.y, r.tool_curr_pose_in_slvrf.position.z, ORANGE);
//
//        		// draw the destination point
//        		dr.drawToolTip(imageCopy,r.tool_dest_pose_in_slvrf.position.x , r.tool_dest_pose_in_slvrf.position.y, r.tool_dest_pose_in_slvrf.position.z, BLUE);

                dr.drawGuidingLines(imageCopy, r.tool_curr_pose_in_slvrf,
                    r.tool_dest_pose_in_slvrf);

                //-----------------------------------------------------------------------------------
                // publish camera to robot pose
                //-----------------------------------------------------------------------------------
                KDL::Frame cam_to_robot = br_frame * bc_frame.Inverse();

                // convert pixel to meters
                cam_to_robot.p = cam_to_robot.p / dr.m_to_px;
                tf::poseKDLToMsg(cam_to_robot, cr_pose_msg);

                pub_cam_to_robot_pose.publish(cr_pose_msg);
                pub_board_to_robot_pose.publish(br_pose_msg);

            }

        }

        //-----------------------------------------------------------------------------------
        // during the task execution
        //-----------------------------------------------------------------------------------

        if (r.acqusition_running) {
            dr.setTransientNotification("Acquiring task data", 50, 60, 2);

            // task started.
            if (dr.new_task) {
                dr.resetUsersPath();
                dr.new_task = false;
            };
            //Record the path
            dr.recordUsersPath(dr.isToolInTouch(-0.001));
        } else {
            // task ended
            dr.new_task = true;

        }

        dr.drawUsersPath(imageCopy);
        dr.showTransientNotification(imageCopy);

        if (r.msg_ws_alert)
            dr.showWorkspaceAlert(imageCopy);

        //-----------------------------------------------------------------------------------
        // Output Text
        //-----------------------------------------------------------------------------------
//        int baseLine = 0;
        Point textOrigin(10, 10);
        putText(imageCopy, instructions, textOrigin, 1, 1, (status == 0) ? RED : GREEN1);

        imshow("teleop", imageCopy);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Ending Session...\n");
    ros::shutdown();

    return 0;
}


void make_tr(vector<Point3d> axisPoints, Matx33d &_rotm, Vec3d &br_tvec) {

    br_tvec = axisPoints[0];

    Vec3f x = Vec3f(axisPoints[1].x - axisPoints[0].x,
        axisPoints[1].y - axisPoints[0].y,
        axisPoints[1].z - axisPoints[0].z);

    Vec3f y = Vec3f(axisPoints[2].x - axisPoints[0].x,
        axisPoints[2].y - axisPoints[0].y,
        axisPoints[2].z - axisPoints[0].z);
//	Vec3f x = Vec3f(-2.0, -0.2, 0.02);
//	Vec3f y = Vec3f(-0.2, -2.0, 0.04);
    cv::normalize(x, x);
    cv::normalize(y, y);
    Vec3f z = x.cross(y);
    cv::normalize(z, z);
    x = y.cross(z);
    y = z.cross(x);
    Matx33d br_rotm = Matx33d::ones();

    _rotm(0, 0) = x[0];
    _rotm(0, 1) = y[0];
    _rotm(0, 2) = z[0];
    _rotm(1, 0) = x[1];
    _rotm(1, 1) = y[1];
    _rotm(1, 2) = z[1];
    _rotm(2, 0) = x[2];
    _rotm(2, 1) = y[2];
    _rotm(2, 2) = z[2];

//	br_rotm.push_back(yy );
//	br_rotm.push_back(zz );
//	br_rotm.t();


//	cout << "x: " << x.val[0] << " "<< x.val[1] << " "<< x.val[2] << endl;
//	cout << "y: " << y.val[0] << " "<< y.val[1] << " "<< y.val[2] << endl;
//	cout << "z: " << z.val[0] << " "<< z.val[1] << " "<< z.val[2] << endl;

}


void drawCube(InputOutputArray _image, InputArray _cameraMatrix, InputArray _distCoeffs,
    InputArray _rvec, InputArray _tvec) {

    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));

    // project axis points
    vector<Point3f> axisPoints;
    double w = 100;
    double h = 100;
    double d = 200;
    double x_start = 250;
    double y_start = 450;
    double z_start = 0;
    axisPoints.push_back(Point3f(x_start, y_start + h, z_start));
    axisPoints.push_back(Point3f(x_start + w, y_start + h, z_start));
    axisPoints.push_back(Point3f(x_start + w, y_start, z_start));
    axisPoints.push_back(Point3f(x_start, y_start, z_start));

    axisPoints.push_back(Point3f(x_start, y_start + h, z_start + d));
    axisPoints.push_back(Point3f(x_start + w, y_start + h, z_start + d));
    axisPoints.push_back(Point3f(x_start + w, y_start, z_start + d));
    axisPoints.push_back(Point3f(x_start, y_start, z_start + d));

    vector<Point2f> imagePoints;
    projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

    // draw axis lines
    int points[7] = {0, 1, 2, 4, 5, 6};
    for (int i = 0; i < 6; i++) {
        line(_image, imagePoints[points[i]], imagePoints[points[i] + 1], Scalar(200, 100, 10), 1);
    }
    for (int i = 0; i < 4; i++) {
        line(_image, imagePoints[i], imagePoints[i + 4], Scalar(200, 100, 10), 1);
    }
    line(_image, imagePoints[3], imagePoints[0], Scalar(200, 100, 10), 1);
    line(_image, imagePoints[7], imagePoints[4], Scalar(200, 100, 10), 1);
}


//-----------------------------------------------------------------------------------
// ROS OBJECT CLASS METHODS
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
// constructor
//-----------------------------------------------------------------------------------
rosObj::rosObj(int argc, char *argv[], string n_name) {
    freq_ros = 0;
    all_good = false;
    camId_param = 0;
    node_name = n_name;
    new_event = false;
    msg_ws_alert = false;
    acqusition_running = false;
}


//-----------------------------------------------------------------------------------
// init
//-----------------------------------------------------------------------------------

void rosObj::init() {
    all_good = true;
    n.param<double>("frequency", freq_ros, 25);

    if (!ros::param::has(node_name + "/cam_data_path")) {
        ROS_ERROR("Parameter cam_data_path is required.");
        all_good = false;
    } else n.getParam(node_name + "/cam_data_path", cam_data_path_param);

    if (!ros::param::has(node_name + "/tool_curr_topic_name")) {
        ROS_ERROR("Parameter tool_curr_topic_name is required.");
        all_good = false;
    } else n.getParam(node_name + "/tool_curr_topic_name", tool_curr_topic_name_param);

    if (!ros::param::has(node_name + "/tool_dest_topic_name")) {
        ROS_ERROR("Parameter tool_dest_topic_name is required.");
        all_good = false;
    } else n.getParam(node_name + "/tool_dest_topic_name", tool_dest_topic_name_param);

    if (!ros::param::has(node_name + "/events_topic_name")) {
        ROS_ERROR("Parameter tool_dest_topic_name is required.");
        all_good = false;
    } else n.getParam(node_name + "/events_topic_name", events_topic_name_param);

    if (!ros::param::has(node_name + "/board_to_robot_tr")) {
        ROS_INFO("Parameter board_to_robot_tr is not set. Calibration is required.");
//		all_good = false;
    } else n.getParam(node_name + "/board_to_robot_tr", board_to_robot_tr_param);

    n.param<int>(node_name + "/arucoMarkersX", markersX_param, 9);
    n.param<int>(node_name + "/arucoMarkersY", markersY_param, 6);
    n.param<float>(node_name + "/arucoMarkerLength_px", markerLength_px_param, 100);
    n.param<float>(node_name + "/arucoMarkerSeparation_px", markerSeparation_px_param, 20);
    n.param<int>(node_name + "/arucoDictionaryId", dictionaryId_param, 9);
    n.param<float>(node_name + "/arucoMarkerLength_m", markerlength_m_param, 0.027);
    n.param<int>(node_name + "/camId", camId_param, 0);

}


//-----------------------------------------------------------------------------------
// toolCurrCallback
//-----------------------------------------------------------------------------------

void rosObj::toolCurrCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    tool_curr_pose_in_slvrf.position = msg->position;
    tool_curr_pose_in_slvrf.orientation = msg->orientation;
//		ros::Duration d(0.01);
//		d.sleep();
}


//-----------------------------------------------------------------------------------
// toolDestCallback
//-----------------------------------------------------------------------------------
void rosObj::toolDestCallback(const geometry_msgs::Pose::ConstPtr &msg) {
    //    ROS_INFO_STREAM("chatter1: [" << msg->position << "] [thread=" << boost::this_thread::get_id() << "]");
    tool_dest_pose_in_slvrf.position = msg->position;
    tool_dest_pose_in_slvrf.orientation = msg->orientation;
//		ros::Duration d(0.01);
//		d.sleep();
}


//-----------------------------------------------------------------------------------
// eventsCallback
//-----------------------------------------------------------------------------------
void rosObj::eventsCallback(const std_msgs::Char::ConstPtr &msg) {
    this->new_event = true;
    this->acquisition_event = *msg;
    cout << "received event: " << msg->data << endl;


    switch (this->acquisition_event.data) {

        case 'w':
            this->msg_ws_alert = true;
            break;

        case 'q':
            this->msg_ws_alert = false;
            break;

        case 's':
            this->acqusition_running = true;
            break;

        case 'e':
            this->acqusition_running = false;
            break;

        case 'd':
            this->acqusition_running = false;
            break;

        default:
            // do nothing
            break;
    }
}


//-----------------------------------------------------------------------------------
// BOARD DETECtOR CLASS METHODS
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
// constructor
//-----------------------------------------------------------------------------------
boardDetector::boardDetector(int _markersX, int _markersY, float _markerLength,
    float _markerSeparation, int _dictionaryId, Mat &_camMatrix,
    Mat &_distCoeffs, double _n_avg) {

    markersX = _markersX;
    markersY = _markersY;
    markerLength = _markerLength;
    markerSeparation = _markerSeparation;
    dictionaryId = _dictionaryId;
    camMatrix = _camMatrix;
    distCoeffs = _distCoeffs;
    axisLength = 200;
    refindStrategy = false;
    markersOfBoardDetected = 0;

    ready = false;
    init_counter = 0;

    detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers
    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    gridboard = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation,
        dictionary);
    board = gridboard.staticCast<aruco::Board>();
    n_avg = _n_avg;
}


//-----------------------------------------------------------------------------------
// detect
//-----------------------------------------------------------------------------------
void boardDetector::detect(Vec3d &_rvec, Vec3d &_tvec) {

    Vec3d rvec_curr, tvec_curr;

    // detect markers
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

    // refind strategy to detect more markers
    if (refindStrategy)
        aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix, distCoeffs);

    // estimate board pose
    if (ids.size() > 0) {
        markersOfBoardDetected =
            aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec_curr,
                tvec_curr);

        // average approximation of the orientation to fix the oscillation of the axes
        rvec -= rvec / n_avg;
        rvec += rvec_curr / n_avg;

        // averaging the origin position
        tvec -= tvec / n_avg;
        tvec += tvec_curr / n_avg;

        // output
        _rvec = rvec;
        _tvec = tvec;

        // wait for the initialization of the averaging (4 times the avg window should be enough)
        if (init_counter < n_avg * 4)
            init_counter++;
        else
            ready = true;
    } else
        ready = false;
}

//-----------------------------------------------------------------------------------
// drawAxis
//-----------------------------------------------------------------------------------
void boardDetector::drawAxis() {
    if (markersOfBoardDetected > 0)
        aruco::drawAxis(image, camMatrix, distCoeffs, rvec, tvec, axisLength);
}

//-----------------------------------------------------------------------------------
// drawDetectedMarkers
//-----------------------------------------------------------------------------------
void boardDetector::drawDetectedMarkers() {
    if (ids.size() > 0)
        aruco::drawDetectedMarkers(image, corners, ids);
}


//-----------------------------------------------------------------------------------
// readCameraParameters
//-----------------------------------------------------------------------------------
bool boardDetector::readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


//-----------------------------------------------------------------------------------
// CALIB BOARD ROBOT CLASS METHODS
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// updateMe
//-----------------------------------------------------------------------------------
bool calibBoardRobot::updateMe(string &msg, cv::Mat img) {


    int pointsGot = axisPoints.size();
    circle(img, visual_calib_points[pointsGot], 5, Scalar(255, 255, 0), 4);
    if (pointsGot < 3) {
        // Need more points. show the message for the next point
        std::ostringstream oss;
        message = "Point at point ";
        oss << pointsGot + 1;
        message += oss.str();
    }
    msg = message;

    return calib_status;
}

//-----------------------------------------------------------------------------------
// setCalibpoint
//-----------------------------------------------------------------------------------
void calibBoardRobot::setCalibpoint(double x, double y, double z) {

    axisPoints.push_back(Point3f(x, y, z));
    cout << "got: " << x << " " << y << " " << z << endl;

    int pointsGot = axisPoints.size();

    if (pointsGot == 3) {

        // got all the 3 points, find the transformation
        make_tr(axisPoints, calib_rotm, calib_tvec);

        // set the status as done
        calib_status = true;
        message = "Calibration Done.";
    }


}

//--------------------------------------------------------------------------------------------
// make_tr
//--------------------------------------------------------------------------------------------
void calibBoardRobot::make_tr(vector<Point3d> axisPoints, Matx33d &_rotm, Vec3d &br_tvec) {

    br_tvec = axisPoints[0];

    Vec3f x = Vec3f(axisPoints[1].x - axisPoints[0].x,
        axisPoints[1].y - axisPoints[0].y,
        axisPoints[1].z - axisPoints[0].z);

    Vec3f y = Vec3f(axisPoints[2].x - axisPoints[0].x,
        axisPoints[2].y - axisPoints[0].y,
        axisPoints[2].z - axisPoints[0].z);
//	Vec3f x = Vec3f(-2.0, -0.2, 0.02);
//	Vec3f y = Vec3f(-0.2, -2.0, 0.04);
    cv::normalize(x, x);
    cv::normalize(y, y);
    Vec3f z = x.cross(y);
    cv::normalize(z, z);
    x = y.cross(z);
    y = z.cross(x);
    z = x.cross(y);
    // just to be sure!
    cv::normalize(x, x);
    cv::normalize(y, y);
    cv::normalize(z, z);

    Matx33d br_rotm = Matx33d::ones();

    _rotm(0, 0) = x[0];
    _rotm(0, 1) = y[0];
    _rotm(0, 2) = z[0];
    _rotm(1, 0) = x[1];
    _rotm(1, 1) = y[1];
    _rotm(1, 2) = z[1];
    _rotm(2, 0) = x[2];
    _rotm(2, 1) = y[2];
    _rotm(2, 2) = z[2];

    cout << "Rotation:" << endl;
    cout << _rotm(0, 0) << " " << _rotm(0, 1) << " " << _rotm(0, 2) << endl;
    cout << _rotm(1, 0) << " " << _rotm(1, 1) << " " << _rotm(1, 2) << endl;
    cout << _rotm(2, 0) << " " << _rotm(2, 1) << " " << _rotm(2, 2) << endl;
    cout << "Translation:" << endl;
    cout << br_tvec[0] << endl;
    cout << br_tvec[1] << endl;
    cout << br_tvec[2] << endl;
//	br_rotm.push_back(yy );
//	br_rotm.push_back(zz );
//	br_rotm.t();


//	cout << "x: " << x.val[0] << " "<< x.val[1] << " "<< x.val[2] << endl;
//	cout << "y: " << y.val[0] << " "<< y.val[1] << " "<< y.val[2] << endl;
//	cout << "z: " << z.val[0] << " "<< z.val[1] << " "<< z.val[2] << endl;

}

void calibBoardRobot::get_tr(Matx33d &_rotm, Vec3d &_br_tvec) {
    _rotm = calib_rotm;
    _br_tvec = calib_tvec;

}

void calibBoardRobot::reset() {
    cout << " initial size of axis points = " << axisPoints.size() << endl;
    calib_status = false;

    while (axisPoints.size() > 0) {
        axisPoints.pop_back();
    }
}


//--------------------------------------------------------------------------------------------
//  AC geometry
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------
drawings::drawings(
    const Mat _camMatrix,
    const Mat _distCoeffs,
    const KDL::Frame _br_frame,
    const double _m_to_px) {

    camMatrix = _camMatrix;
    distCoeffs = _distCoeffs;

    br_trans = Point3d(_br_frame.p[0], _br_frame.p[1], _br_frame.p[2]);
    conversions::kdlRotToMatx33d(_br_frame.M, br_rotm);

    m_to_px = _m_to_px;
    new_task = true;
    notification_counter = 0;

}

void drawings::drawSquare(InputOutputArray _image) {

    CV_Assert(_image.getMat().total() != 0 &&
              (_image.getMat().channels() == 1 || _image.getMat().channels() == 3));

    // project axis points
    sq_points_in_board.push_back(
        Point3f(sq_center.x - sq_dims.x / 2, sq_center.y - sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x + sq_dims.x / 2, sq_center.y - sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x + sq_dims.x / 2, sq_center.y + sq_dims.y / 2, sq_center.z));
    sq_points_in_board.push_back(
        Point3f(sq_center.x - sq_dims.x / 2, sq_center.y + sq_dims.y / 2, sq_center.z));

    vector<Point2d> imagePoints;
    projectPoints(sq_points_in_board, bc_rvec, bc_tvec, camMatrix, distCoeffs, imagePoints);

    // draw axis lines
    for (int i = 0; i < 3; i++) {
        line(_image, imagePoints[i], imagePoints[i + 1], Scalar(200, 100, 10), 1);
    }
    line(_image, imagePoints[3], imagePoints[0], Scalar(200, 100, 10), 1);

}


void drawings::createEllipse(double _center_x, double _center_y, double a, double b,
    unsigned int n_points) {

    while (ac_points_in_board.size() > 0) {
        ac_points_in_board.pop_back();
        ac_points_in_robot.pop_back();
    }

    for (unsigned int i = 0; i < n_points; i++) {
        double t = M_PI / 2 + double(i) / double(n_points) * M_PI;
        ac_points_in_board.push_back(
            Point3f(_center_x + a * sin(t), _center_y + b * cos(t), -0.014 * m_to_px));
        ac_points_in_robot.push_back(br_rotm * (ac_points_in_board[i] / m_to_px) + br_trans);
    }

}


void drawings::createSine(double _center_x, double _center_y, double a, double b,
    unsigned int n_points) {

    while (ac_points_in_board.size() > 0) {
        ac_points_in_board.pop_back();
        ac_points_in_robot.pop_back();
    }

    for (unsigned int i = 0; i < n_points; i++) {
        double x = double(i) / double(n_points);
        ac_points_in_board.push_back(
            Point3f(_center_x - a / 2 + a * x, _center_y + b * sin(x * 2 * M_PI),
                -0.014 * m_to_px));
        ac_points_in_robot.push_back(br_rotm * (ac_points_in_board[i] / m_to_px) + br_trans);
    }

//    // calculate the length of the path
//    double circum = 0.0;
//    for(unsigned int i=1; i<n_points ; i++){
//    	Point3f d = (ac_points_in_robot[i] - ac_points_in_robot[i-1]);
//    	circum+= sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
//    }
//    cout << "Sine circumference = " << circum << endl;
}


void
drawings::createHat(double _center_x, double _center_y, double a, double b, unsigned int n_points) {

    while (ac_points_in_board.size() > 0) {
        ac_points_in_board.pop_back();
        ac_points_in_robot.pop_back();
    }

    for (unsigned int i = 0; i < n_points; i++) {
        double x = double(i) / double(n_points);

        double y = a - b * 2 * x;
        if (i > n_points / 2) y = a - 2 * b + b * 2 * x;

        ac_points_in_board.push_back(Point3f(_center_x - a / 2 + a * x, y, -0.014 * m_to_px));
        ac_points_in_robot.push_back(br_rotm * (ac_points_in_board[i] / m_to_px) + br_trans);
    }
}


void drawings::create3dCurve(double center_x, double center_y, unsigned int n_points) {

    double dx = 100;
    double dy = 100;
    double dz = 50;

    while (ac_points_in_board.size() > 0) {
        ac_points_in_board.pop_back();
    }

    for (unsigned int i = 0; i < n_points; i++) {
        double t = double(i) / double(n_points) * M_PI * 2;
        ac_points_in_board.push_back(
            Point3f(center_x + dx * sin(t), center_y + dy * cos(t), dz * sin(2 * t)));
        ac_points_in_robot.push_back(br_rotm * (ac_points_in_board[i] / m_to_px + br_trans));
    }
}


void drawings::drawACPoints(InputOutputArray _image, bool _is_tool_in_touch) {

    Scalar color;
    if (_is_tool_in_touch)
        color = GREEN1;
    else
        color = BLUE2;

    // project  points
    vector<Point2d> imagePoints;
    if (ac_points_in_board.size() > 0) {
        projectPoints(ac_points_in_board, bc_rvec, bc_tvec, camMatrix, distCoeffs, imagePoints);

        // draw points
        for (unsigned int i = 0; i < imagePoints.size(); i++) {
            circle(_image, imagePoints[i], 2, color, -1);
        }

        // draw the start and end points


    }
}


void drawings::drawToolTip(InputOutputArray _image, double _x, double _y, double _z,
    const Scalar _color) {

//	Point3d br_trans = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
    Point3d toolPoint3d_rrf = Point3d(_x, _y, _z);

    // taking the robot tool tip from the robot ref frame to board ref frame and convert to pixles from meters
    Point3d temp = br_rotm.t() * (toolPoint3d_rrf - br_trans);
    Point3d toolPoint3d_crf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

    vector<Point3d> toolPoint3d_vec_crf;
    vector<Point2d> toolPoint2d;
    toolPoint3d_vec_crf.push_back(toolPoint3d_crf);

    projectPoints(toolPoint3d_vec_crf, bc_rvec, bc_tvec, camMatrix, distCoeffs, toolPoint2d);
    circle(_image, toolPoint2d[0], 2, _color, -1);

}


void drawings::drawGuidingLines(InputOutputArray _image, const geometry_msgs::Pose &tool,
    const geometry_msgs::Pose &desired) {


    // projecting the tool on the board by setting the z = 0
    Point3d projectedToolPoint3d_crf = curr_tool_boardrf;
    projectedToolPoint3d_crf.z = 0.0;

    vector<Point3d> point3d_vec_crf;
    vector<Point2d> point2d_vec;
    point3d_vec_crf.push_back(curr_tool_boardrf);
    point3d_vec_crf.push_back(dest_tool_boardrf);
    point3d_vec_crf.push_back(projectedToolPoint3d_crf);

    projectPoints(point3d_vec_crf, bc_rvec, bc_tvec, camMatrix, distCoeffs, point2d_vec);


    line(_image, point2d_vec[0], point2d_vec[2], GREEN1, 1);
    line(_image, point2d_vec[1], point2d_vec[2], GREEN2, 1);

}


//--------------------------------------------------------------------------------------------
// Get cloud points of the ac
//--------------------------------------------------------------------------------------------
void drawings::getACCloud(sensor_msgs::PointCloud2 &cloud_out) {

    int n_points = ac_points_in_robot.size();

    // Fill some internals of the PoinCloud2 like the header/width/height ...
    cloud_out.height = 1;
    cloud_out.width = 3;

    // Set the point fields to xyzrgb and resize the vector with the following command
    // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
    // the number of occurences of the type in the PointField, the type of the PointField
    sensor_msgs::PointCloud2Modifier modifier(cloud_out);

    modifier.setPointCloud2Fields(
        3,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32);

    // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
    // You have to be aware that the following function does add extra padding for backward compatibility though
    // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
    // 2 is for the number of fields to add
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // You can then reserve / resize as usual
    modifier.resize(n_points);

    // Define the iterators. When doing so, you define the Field you would like to iterate upon and
    // the type of you would like returned: it is not necessary the type of the PointField as sometimes
    // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");

    // Fill the PointCloud2
    for (size_t i = 0; i < n_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = ac_points_in_robot[i].x;
        *iter_y = ac_points_in_robot[i].y;
        *iter_z = ac_points_in_robot[i].z;
    }
}


//--------------------------------------------------------------------------------------------
// Get cloud points of the ac
//--------------------------------------------------------------------------------------------
void drawings::transfromToBoard(const geometry_msgs::Pose &_tool_curr_in_slvrf,
    const geometry_msgs::Pose &_tool_dest_in_slvrf) {

    //	Point3d br_trans = Point3d( br_frame.p[0],  br_frame.p[1],  br_frame.p[2]);
    Point3d toolPoint3d_slvrf = Point3d(_tool_curr_in_slvrf.position.x,
        _tool_curr_in_slvrf.position.y, _tool_curr_in_slvrf.position.z);
    Point3d desPoint3d_slvrf = Point3d(_tool_dest_in_slvrf.position.x,
        _tool_dest_in_slvrf.position.y, _tool_dest_in_slvrf.position.z);

    // taking the robot tool tip from the robot ref frame to board ref frame and convert to pixels from meters
    Point3d temp = br_rotm.t() * (toolPoint3d_slvrf - br_trans);
    curr_tool_boardrf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

    temp = br_rotm.t() * (desPoint3d_slvrf - br_trans);
    dest_tool_boardrf = Point3d(temp.x * m_to_px, temp.y * m_to_px, temp.z * m_to_px);

}


//--------------------------------------------------------------------------------------------
// recordUsersPath
//--------------------------------------------------------------------------------------------
void drawings::recordUsersPath(bool _is_tool_in_touch) {

    if (_is_tool_in_touch) {
        // save the point coordinates
        users_path.push_back(curr_tool_boardrf);

        // find the distance
//		double dis =  sqrt(
//				pow((dest_tool_boardrf.x - curr_tool_boardrf.x),2) +
//				pow((dest_tool_boardrf.y - curr_tool_boardrf.y),2) +
//				pow((dest_tool_boardrf.z - curr_tool_boardrf.z),2)  ) /m_to_px;

        double penet = (dest_tool_boardrf.z - curr_tool_boardrf.z) / m_to_px;
        // when the user penetrates more (z<0) turn the color more red
        double dis_max = 0.005;
        if (penet < 0.0)
            penet = 0.0;
        else if (penet > dis_max)
            penet = dis_max;
//		std::cout << "penet: " << penet << std::endl;
//		std::cout << "color: " << 		Scalar( (1-penet/dis_max)*255, 20, (penet/dis_max)*255 )
// << std::endl;
        users_path_colors.push_back(
            Scalar((1 - penet / dis_max) * 255, 20, (penet / dis_max) * 255));
    }

}

//--------------------------------------------------------------------------------------------
// drawUsersPath
//--------------------------------------------------------------------------------------------
void drawings::drawUsersPath(InputOutputArray _image) {

    vector<Point2d> toolPoint2d;

    if (users_path.size() > 0) {
        // convert all the 3d points. Note that we could save the 2d points vector and prevent
        // doing the conversion for all the points every time, but then if the camera moves,
        // the 2d points won't be updates.
        projectPoints(users_path, bc_rvec, bc_tvec, camMatrix, distCoeffs, toolPoint2d);
        circle(_image, toolPoint2d[0], 2, RED, -1);
        for (int i = 0; i < (toolPoint2d.size()); i++) {
            circle(_image, toolPoint2d[i], 1, users_path_colors[i], -1);
        }
    }

}


//--------------------------------------------------------------------------------------------
// resetUsersPath
//--------------------------------------------------------------------------------------------
void drawings::resetUsersPath() {

    while (users_path.size() > 0) {
        users_path.pop_back();
    }

    while (users_path_colors.size() > 0) {
        users_path_colors.pop_back();
    }
}


//--------------------------------------------------------------------------------------------
// setTransientNotification
//--------------------------------------------------------------------------------------------
void drawings::setTransientNotification(string _msg, double _origin_x, double _origin_y,
    unsigned int _duration_counts) {

    textOrigin = Point(_origin_x, _origin_y);
    transient_notification_msg = _msg;
    notification_counter = _duration_counts;


}

//--------------------------------------------------------------------------------------------
// showTransientNotification
//--------------------------------------------------------------------------------------------
void drawings::showTransientNotification(InputOutputArray _image) {


    if (notification_counter > 0) {
        putText(_image, transient_notification_msg, textOrigin, 1, 1, BLUE1);
        notification_counter--;
    }
}


//--------------------------------------------------------------------------------------------
// showTransientNotification
//--------------------------------------------------------------------------------------------
void drawings::showWorkspaceAlert(InputOutputArray _image) {

//	string msg = "WS limit: Release the Clutch";
//	putText( _image, msg, Point(150, 60), FONT_HERSHEY_SIMPLEX, 1.0, RED, 2);

    string text = "Reaching WS limit!";
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.8;
    int thickness = 1;

    int baseline = 0;
    Size textSize = getTextSize(text, fontFace,
        fontScale, thickness, &baseline);

    // center the text

    Point textOrg((_image.cols() - textSize.width) / 2,
        (70 + textSize.height) / 2);

    // draw the box
    rectangle(_image,
        textOrg + Point(-15, baseline),
        textOrg + Point(textSize.width + 15, -textSize.height - 10),
        RED, -1);


    // then put the text itself
    putText(_image, text, textOrg, fontFace, fontScale,
        Scalar::all(255), thickness, 8);

}


//--------------------------------------------------------------------------------------------
// isToolInTouch
//--------------------------------------------------------------------------------------------
bool drawings::isToolInTouch(double _depth) {
    if ((dest_tool_boardrf.z - curr_tool_boardrf.z) / m_to_px > _depth)
        return true;
    else
        return false;
}


//--------------------------------------------------------------------------------------------
//  CONVERSIONS
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------------------
void conversions::rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation &_kdl) {
    cv::Matx33d mat;
    cv::Rodrigues(_rvec, mat);
    _kdl = KDL::Rotation(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2),
        mat(2, 0), mat(2, 1), mat(2, 2));
}

void
conversions::rvectvecToKdlFrame(const cv::Vec3d _rvec, const cv::Vec3d _tvec, KDL::Frame &_kdl) {
    cv::Matx33d mat;
    cv::Rodrigues(_rvec, mat);
    _kdl.M = KDL::Rotation(mat(0, 0), mat(0, 1), mat(0, 2), mat(1, 0), mat(1, 1), mat(1, 2),
        mat(2, 0), mat(2, 1), mat(2, 2));
    _kdl.p.data[0] = _tvec.val[0];
    _kdl.p.data[1] = _tvec.val[1];
    _kdl.p.data[2] = _tvec.val[2];
}


void conversions::matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation &_kdl) {
    _kdl = KDL::Rotation(_mat(0, 0), _mat(0, 1), _mat(0, 2), _mat(1, 0), _mat(1, 1), _mat(1, 2),
        _mat(2, 0), _mat(2, 1), _mat(2, 2));
}


void conversions::kdlRotToMatx33d(const KDL::Rotation _kdl, cv::Matx33d &_mat) {
    _mat = cv::Matx33d(_kdl(0, 0), _kdl(0, 1), _kdl(0, 2), _kdl(1, 0), _kdl(1, 1), _kdl(1, 2),
        _kdl(2, 0), _kdl(2, 1), _kdl(2, 2));
}

void conversions::poseMsgToVector(const geometry_msgs::Pose in_pose, vector<double> &out_vec) {

    out_vec.at(0) = in_pose.position.x;
    out_vec.at(1) = in_pose.position.y;
    out_vec.at(2) = in_pose.position.z;
    out_vec.at(3) = in_pose.orientation.x;
    out_vec.at(4) = in_pose.orientation.y;
    out_vec.at(5) = in_pose.orientation.z;
    out_vec.at(6) = in_pose.orientation.w;
}

void conversions::vectorToPoseMsg(const vector<double> in_vec, geometry_msgs::Pose &out_pose) {

    out_pose.position.x = in_vec.at(0);
    out_pose.position.y = in_vec.at(1);
    out_pose.position.z = in_vec.at(2);
    out_pose.orientation.x = in_vec.at(3);
    out_pose.orientation.y = in_vec.at(4);
    out_pose.orientation.z = in_vec.at(5);
    out_pose.orientation.w = in_vec.at(6);

}


// operator overload to print out vectors
ostream &operator<<(ostream &out, const vector<double> &vect) {
    for (unsigned int iter = 0; iter < vect.size(); ++iter) {
        out << "[" << iter << "]: " << vect.at(iter) << "\t";
    }

    return out;
}
