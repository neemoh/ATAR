#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"
#include "OverlayGraphics.h"
#include <std_msgs/Float32.h>


#include <iostream>
//#include <GLFW/glfw3.h>
//#include <GL/glu.h>

#include "opencv2/highgui/highgui.hpp"




cv::Mat image_left_rect;

cv::Mat imgR;
cv::Mat image_right_rect;

cv::Mat cameraMatrixL;
cv::Mat distCoeffL;
cv::Mat cameraMatrixR;
cv::Mat distCoeffR;
cv::Mat Rstereo;
cv::Mat tstereo;
cv::Mat R1;
cv::Mat R2;
KDL::Frame Tstereo;
KDL::Frame R1_kdl;
KDL::Frame R2_kdl;
KDL::Frame cameraPoseLeft;
KDL::Frame cameraPoseRight;
int dWidth = 0; //get the width of frames of the video
int dHeight = 0;
cv::Mat bufferLeft;
cv::Mat bufferRight;
//cv::Mat gauge[11];
//GLuint gaugeTexId[11];
bool enableHud = true;
bool enableImage = true;

int frame = 0;

cv::Point2f point;
std::vector<cv::Point2f> safety_area;


TabletInfo tablet_info;


//void bb_coords_Callback(const enVisors2::bb_coords& bb)
//{
//	bb_coord[0] = bb.bb_coords[0];
//	bb_coord[1] = bb.bb_coords[1];
//	bb_coord[2] = bb.bb_coords[2];
//	bb_coord[3] = bb.bb_coords[3];
//	bb_coord[4] = bb.bb_coords[4];
//	bb_coord[5] = bb.bb_coords[5];
//}

//void distance_Callback(const std_msgs::Float32Ptr& dist)
//{
//	distance = dist->data;
//}




void imageLeft_rectCallback(const sensor_msgs::ImageConstPtr& img)
{
    int frame  = img->header.seq;
//	fromImagetoMat(*img, &image_left_rect, "bgr8");
}

void imageRight_rectCallback(const sensor_msgs::ImageConstPtr& img)
{
    int frame  = img->header.seq;
//	fromImagetoMat(*img, &image_right_rect, "bgr8");
}



int main(int argc, char **argv)
{

    ros::init(argc, argv, "aug");



    //	std::string path = CONFIG_FILE_FOLDER;

//	ros::Subscriber sub = n.subscribe("/camera/pose", 1, cameraPoseCallback);
//	image_transport::Subscriber image_left_rect_sub = it.subscribe("/image/left/rectified", 1, &imageLeft_rectCallback);
//	image_transport::Subscriber image_right_rect_sub = it.subscribe("/image/right/rectified", 1, &imageRight_rectCallback);
//	ros::Subscriber bb_sub = n.subscribe("/bb/coords", 1000, bb_coords_Callback);
//	ros::Subscriber distance_sub = n.subscribe("/bb/distance", 1000, distance_Callback);
//	ros::Subscriber tablet_sub = n.subscribe("/tablet/info", 1000, tablet_callback);
//	image_transport::Publisher image_left_pub = it.advertise("/near/aug/left", 1);
//	image_transport::Publisher image_right_pub = it.advertise("/near/aug/right", 1);
//	int framePrev = 0;
//
//	// read intrinsic params camera left
//	readParamsCamera(path  + "camera/davinci/LeftCameraParameters.xml",
//					 cameraMatrixL,
//					 distCoeffL);
//	// read intrinsic params camera right
//	readParamsCamera(path  + "camera/davinci/RightCameraParameters.xml",
//					 cameraMatrixR,
//					 distCoeffR);
//	// read intrinsic params camera right
//	readExtrisicParamsCamera(
//			path  + "camera/davinci/StereoCameraParameters.xml",
//			Rstereo,
//			tstereo);
//
//	//Read Rotation Matrix after rectification camera left
//	readRectificationMatrix(
//			path  + "camera/davinci/R1.xml",
//			R1);
//	//Read Rotation Matrix after rectification camera left
//	readRectificationMatrix(
//			path  + "camera/davinci/R2.xml",
//			R2);


//	cv::Mat t(3,1, CV_64FC1);
//	t.setTo(0.0);
//
//	// convert Matrices to KDL
//	conversions::RvecTvecToKDLFrame(Rstereo, tstereo, Tstereo);
//	conversions::RvecTvecToKDLFrame(R1, t, R1_kdl);
//	conversions::RvecTvecToKDLFrame(R2, t, R2_kdl);
//	//	std::cout<< "ML: " <<cameraMatrixL <<std::endl;
//	//	std::cout<< "DS: " <<distCoeffL <<std::endl;
//
//	char fileNameGauge[256];

//
//	//load gauge images
//	for(int i = 0; i <=10; i++)
//	{
//		sprintf(fileNameGauge, "%s/gaugeNew/gauge%d.png", path.c_str(), i);
//		//std::cout << "gauge image: " << fileNameGauge << std::endl;
//		gauge[i] = imread(fileNameGauge, -1);
//	}

//	dWidth = 720;
//	dHeight = 576;
//	dWidth = imgL.cols;
//	dHeight = imgL.rows;
//
//	//opengl
//	GLFWwindow* window;
//
//
//Share
//	/* Initialize the library */
//	if (!glfwInit())
//		return -1;
//
//	/* Create a windowed mode window and its OpenGL context */
//	window = glfwCreateWindow(dWidth * 2, dHeight, "Augmented Reality", NULL, NULL);
//	if (!window)
//	{
//		glfwTerminate();
//		return -1;
//	}
//
//	/* Make the window's context current */
//	glfwMakeContextCurrent(window);
//	og.InitGL(dWidth, dHeight );


    OverlayGraphics og (ros::this_node::getName(),720, 576);

    ros::Rate loop_rate(og.ros_freq);
    // Create the window in which to render the video feed
    cvNamedWindow("Overlay Left",CV_WINDOW_NORMAL);

//    while (ros::ok() && !glfwWindowShouldClose(window))
    while (ros::ok())
    {
        // --------------------------------------------------------------------------------------
        // keyboard commands
        std::string left_window_name = "Overlay Left";
        std::string right_window_name = "Overlay Right";

        char key = (char)cv::waitKey(1);
        if (key == 27) // Esc
            ros::shutdown();
        else if (key == 'f')  //full screen
            VisualUtils::SwitchFullScreen(left_window_name);



        // --------------------------------------------------------------------------------------
        // Draw things
        og.DrawCube(og.ImageLeft(), og.Camera,
                    og.cam_rvec_l, og.cam_tvec_l);

        og.DrawToolTip(og.ImageLeft(), og.Camera, og.cam_rvec_l, og.cam_tvec_l,
                       og.pose_psm2.p, cv::Scalar(100, 50, 200));

        cv::imshow("Overlay Left", og.ImageLeft());

//        std::cout << og.pose_psm2.p[0] << std::endl;

//        cv::imshow("Aruco extrinsic", og.ImageLeft());



//		if(!image_left_.empty() && !image_right_.empty())
//		{
//
//			if (tablet_info.buttons[0] && !image_left_rect.empty() && !image_right_rect.empty() )
//			{
//				// Copy frame to texture
//				glBindTexture(GL_TEXTURE_2D, og.texIdL);
//				glTexSubImage2D(
//						GL_TEXTURE_2D, 0, 0, 0,
//						dWidth, dHeight,
//						GL_BGR, GL_UNSIGNED_BYTE, image_left_rect.data);
//
//				// Copy frame to texture
//				glBindTexture(GL_TEXTURE_2D, og.texIdR);
//				glTexSubImage2D(
//						GL_TEXTURE_2D, 0, 0, 0,
//						dWidth, dHeight,
//						GL_BGR, GL_UNSIGNED_BYTE, image_right_rect.data);
//			}
//			else
//			{
//				// Copy frame to texture
//				glBindTexture(GL_TEXTURE_2D, og.texIdL);
//				glTexSubImage2D(
//						GL_TEXTURE_2D, 0, 0, 0,
//						dWidth, dHeight,
//						GL_BGR, GL_UNSIGNED_BYTE, image_left_.data);
//
//				// Copy frame to texture
//				glBindTexture(GL_TEXTURE_2D, og.texIdR);
//				glTexSubImage2D(
//						GL_TEXTURE_2D, 0, 0, 0,
//						dWidth, dHeight,
//						GL_BGR, GL_UNSIGNED_BYTE, image_right_.data);
//			}
//			/* Do the render */
//			og.Render(window, tablet_info, safety_area);
//
//			/* Poll for and process events */
//			glfwPollEvents();
//
//			cv::Mat augL(dHeight, dWidth, CV_8UC4, og.bufferL_);
//			cv::Mat augR(dHeight, dWidth, CV_8UC4, og.bufferR_);
//			//			std::cout << "cols: " << augL.cols << std::endl;
//			//			std::cout << "rows: " << augL.rows << std::endl;
//
//			flip(augL, augL, 0);
//			flip(augR, augR, 0);
//
//			if(enableImage)
//			{
//				cv::putText(augL, "Define Safety Area", cv::Point(60,100), cv::FONT_HERSHEY_PLAIN,
//							4.0, cv::Scalar(255,0,0), 3);
//			}
//
//			// save images
////			if (frame != framePrev)
////			{
////				char frame_number_char[32];
////				sprintf(frame_number_char, "%06d", frame);
////				cv::imwrite(std::string(CONFIG_FILE_FOLDER) + "images/" + "_Laug_" + frame_number_char + ".jpg", augL);
////			}
//			framePrev = frame;
//
//			fromMattoImageA(augL, &image_left_);
//			fromMattoImageA(augR, &image_right_);
//
//
//			image_left_pub.publish(image_left_);
//			image_right_pub.publish(image_right_);
//
//
//		}
//		else
//			std::cout << "Cannot read a frame from video stream" << std::endl;


        ros::spinOnce();
        loop_rate.sleep();
    }

//	glfwDestroyWindow(window);
//	glfwTerminate();
    return 0;
}





