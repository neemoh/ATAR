#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "utils/Conversions.hpp"

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>



#include <iostream>
#include <GLFW/glfw3.h>
#include <GL/glu.h>

#include "opencv2/highgui/highgui.hpp"
// My Library

#include <cv_bridge/cv_bridge.h>


#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>

//#include"enVisorsLib/common.hpp"
//#include"enVisorsLib/paths.h"
//#include <enVisors2/bb_coords.h>

using namespace cv;


cv::Mat imgL;
cv::Mat imgL_rect;
GLuint texIdL;
cv::Mat imgR;
cv::Mat imgR_rect;
GLuint texIdR;
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
float distance = 0.0;
float bb_coord[6];
double dWidth = 0.0; //get the width of frames of the video
double dHeight = 0.0;
cv::Mat bufferLeft;
cv::Mat bufferRight;
//cv::Mat gauge[11];
//GLuint gaugeTexId[11];
bool enableHud = true;
bool enableImage = true;

int frame = 0;

cv::Point2f point;
std::vector<cv::Point2f> safety_area;


unsigned char* bufferL_;
unsigned char* bufferR_;


typedef struct
{
	unsigned char buttons[8];
	float x;
	float y;
	float z;
}TabletInfo;

TabletInfo tablet_info;

//per triangolo
void render(GLFWwindow* window);
void initGL(int w, int h);

void fromMattoImageA(
		const cv::Mat& img_in,
		sensor_msgs::Image* img_out)
{
	cv_bridge::CvImage pics;
	pics.encoding = sensor_msgs::image_encodings::RGBA8 ;
	pics.image = img_in;
	pics.toImageMsg(*img_out);
	img_out->header.stamp = ros::Time::now();
	img_out->header.frame_id = "depth_image";
	img_out->encoding = pics.encoding;
	img_out->width = img_in.cols;
	img_out->height = img_in.rows;
}

void tablet_callback(const sensor_msgs::Joy::ConstPtr& t)
{
	tablet_info.x = t->axes[0] * 2.0f - 1.0f;
	tablet_info.y = t->axes[1] * 2.0f - 1.0f;
	tablet_info.z = t->axes[2];
	tablet_info.buttons[0] = t->buttons[0];
	tablet_info.buttons[1] = t->buttons[1];
	tablet_info.buttons[2] = t->buttons[2];
	tablet_info.buttons[3] = t->buttons[3];
	tablet_info.buttons[4] = t->buttons[4];
	tablet_info.buttons[5] = t->buttons[5];
	tablet_info.buttons[6] = t->buttons[6];
	tablet_info.buttons[7] = t->buttons[7];

	static unsigned char buttonEnablePrev = 0;
	if (tablet_info.buttons[2] && !buttonEnablePrev)
		enableHud = !enableHud;
	buttonEnablePrev = tablet_info.buttons[2];

	if (tablet_info.buttons[0])
	{
		enableImage = true;
	}
	else
	{
		enableImage = false;
	}

	static unsigned char button_stylus_prev = 0;

	if(!tablet_info.buttons[6] && button_stylus_prev)
	{
		safety_area.clear();
	}
	// save safety area contour
	if(tablet_info.buttons[0] && tablet_info.buttons[6])
	{
		point.x = tablet_info.x;
		point.y = tablet_info.y;
		safety_area.push_back(point);
	}

	button_stylus_prev = tablet_info.buttons[6];
}

//void bb_coords_Callback(const enVisors2::bb_coords& bb)
//{
//	bb_coord[0] = bb.bb_coords[0];
//	bb_coord[1] = bb.bb_coords[1];
//	bb_coord[2] = bb.bb_coords[2];
//	bb_coord[3] = bb.bb_coords[3];
//	bb_coord[4] = bb.bb_coords[4];
//	bb_coord[5] = bb.bb_coords[5];
//}

void distance_Callback(const std_msgs::Float32Ptr& dist)
{
	distance = dist->data;
}

void cameraPoseCallback(const geometry_msgs::PoseStampedPtr& t_s)
{
	geometry_msgs::Pose CameraStreamPose;
	CameraStreamPose.orientation = t_s->pose.orientation;
	CameraStreamPose.position = t_s->pose.position;
	tf::poseMsgToKDL(CameraStreamPose, cameraPoseLeft);

	// Calculate position of rectified left and right cameras
	cameraPoseRight = cameraPoseLeft * R1_kdl * Tstereo * R2_kdl.Inverse();
	//cameraPoseLeft = cameraPoseLeft * R1_kdl;
}

void imageLeftCallback(const sensor_msgs::ImageConstPtr& img)
{
	frame  = img->header.seq;
//	fromImagetoMat(*img, &imgL, "bgr8");
}

void imageRightCallback(const sensor_msgs::ImageConstPtr& img)
{
	//int frame  = img->header.seq;
//	fromImagetoMat(*img, &imgR, "bgr8");
}

void imageLeft_rectCallback(const sensor_msgs::ImageConstPtr& img)
{
	int frame  = img->header.seq;
//	fromImagetoMat(*img, &imgL_rect, "bgr8");
}

void imageRight_rectCallback(const sensor_msgs::ImageConstPtr& img)
{
	int frame  = img->header.seq;
//	fromImagetoMat(*img, &imgR_rect, "bgr8");
}

void drawBoundinBox(
		float x_min, float x_max,
		float y_min, float y_max,
		float z_min, float z_max)
{
	glColor4f(0.0f, 0.8f, 0.0f, 0.5f);

	glBegin(GL_QUADS);
	// top
	glNormal3f(0.0f, 1.0f, 0.0f);
	glVertex3f(x_min, y_max, z_max);
	glVertex3f(x_max, y_max, z_max);
	glVertex3f(x_max, y_max, z_min);
	glVertex3f(x_min, y_max, z_min);
	glEnd();

	glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
	glBegin(GL_QUADS);
	// front
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(x_max, y_min, z_max);
	glVertex3f(x_max, y_max, z_max);
	glVertex3f(x_min, y_max, z_max);
	glVertex3f(x_min, y_min, z_max);
	glEnd();

	glColor4f(0.0f, 0.8f, 0.0f, 0.5f);
	glBegin(GL_QUADS);
	// right
	glNormal3f(1.0f, 0.0f, 0.0f);
	glVertex3f(x_max, y_max, z_min);
	glVertex3f(x_max, y_max, z_max);
	glVertex3f(x_max, y_min, z_max);
	glVertex3f(x_max, y_min, z_min);

	glEnd();

	glBegin(GL_QUADS);
	// left
	glNormal3f(-1.0f, 0.0f, 0.0f);
	glVertex3f(x_min, y_min, z_max);
	glVertex3f(x_min, y_max, z_max);
	glVertex3f(x_min, y_max, z_min);
	glVertex3f(x_min, y_min, z_min);

	glEnd();

	glBegin(GL_QUADS);
	// bottom
	glNormal3f(0.0f, -1.0f, 0.0f);
	glVertex3f(x_max, y_min, z_max);
	glVertex3f(x_min, y_min, z_max);
	glVertex3f(x_min, y_min, z_min);
	glVertex3f(x_max, y_min, z_min);

	glEnd();

	glBegin(GL_QUADS);
	// back
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(x_max, y_max, z_min);
	glVertex3f(x_max, y_min, z_min);
	glVertex3f(x_min, y_min, z_min);
	glVertex3f(x_min, y_max, z_min);

	glEnd();
}

void drawElipsoid(
		float x_min, float x_max,
		float y_min, float y_max,
		float z_min, float z_max)
{
	static bool firstTime = true;
	static const int Ns = 40;
	static const int Np = 20;
	static float vertices[(Ns + 1) * (Np * 2 - 1) * 3];

	float *vertex;
	float x, y, z;
	float cosbeta;
	float aux;

	if (firstTime)
	{
		firstTime = false;

		for (int j = 0; j < Np * 2 - 1; j++)
		{
			aux = (float)(j - Np + 1) / (float)Np * M_PI * 0.5f;
			z = sinf(aux);
			cosbeta = cosf(aux);

			for (int i = 0; i < Ns; i++)
			{
				aux = (float)i * 2.0f * M_PI / (float)Ns;
				x = cosbeta * cosf(aux);
				y = -cosbeta * sinf(aux);
				vertex = &vertices[(j * (Ns + 1) + i) * 3];
				vertex[0] = x;
				vertex[1] = y;
				vertex[2] = z;
			}

			vertex = &vertices[(j * (Ns + 1) + Ns) * 3];
			vertex[0] = cosbeta;
			vertex[1] = 0.0f;
			vertex[2] = z;
		}
	}

	glPushMatrix();
	glTranslatef(
			0.5f * (x_max + x_min),
			0.5f * (y_max + y_min),
			0.5f * (z_max + z_min));
	glScalef(
			0.5f * (x_max - x_min),
			0.5f * (y_max - y_min),
			0.5f * (z_max - z_min));
	for (int j = 0; j < Np * 2 - 2; j++)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for (int i = 0; i <= Ns; i++)
		{
			vertex = &vertices[(j * (Ns + 1) + i) * 3];
			glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
			glNormal3f(vertex[0], vertex[1], vertex[2]);
			glVertex3f(vertex[0], vertex[1], vertex[2]);

			vertex = &vertices[((j + 1) * (Ns + 1) + i) * 3];
			glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
			glNormal3f(vertex[0], vertex[1], vertex[2]);
			glVertex3f(vertex[0], vertex[1], vertex[2]);
		}
		glEnd();
	}

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(0.0f, 1.0f, 0.0f, 0.5f);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 1.0f);
	for (int i = Ns; i >= 0; i--)
	{
		vertex = &vertices[((Np * 2 - 2) * (Ns + 1) + i) * 3];
		glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
		glNormal3f(vertex[0], vertex[1], vertex[2]);
		glVertex3f(vertex[0], vertex[1], vertex[2]);
	}
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(0.0f, 0.0f, 0.0f, 0.5f);
	glNormal3f(0.0f, 0.0f, -1.0f);
	glVertex3f(0.0f, 0.0f, -1.0f);
	for (int i = 0; i <= Ns; i++)
	{
		vertex = &vertices[i * 3];
		glColor4f(0.0f, 0.5f * (vertex[2] + 1.0f), 0.0f, 0.5f);
		glNormal3f(vertex[0], vertex[1], vertex[2]);
		glVertex3f(vertex[0], vertex[1], vertex[2]);
	}
	glEnd();

	glPopMatrix();
}

int main(int argc, char **argv)
{

//	std::string path = CONFIG_FILE_FOLDER;

	ros::init(argc, argv, "aug");

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	ros::Subscriber sub = n.subscribe("/camera/pose", 1000, cameraPoseCallback);
	image_transport::Subscriber imgL_rect_sub = it.subscribe("/image/left/rectified", 1, &imageLeft_rectCallback);
	image_transport::Subscriber imgR_rect_sub = it.subscribe("/image/right/rectified", 1, &imageRight_rectCallback);
	image_transport::Subscriber imgL_sub = it.subscribe("/iit/ecm/left", 1, &imageLeftCallback);
	image_transport::Subscriber imgR_sub = it.subscribe("/iit/ecm/right", 1, &imageRightCallback);
//	ros::Subscriber bb_sub = n.subscribe("/bb/coords", 1000, bb_coords_Callback);
	ros::Subscriber distance_sub = n.subscribe("/bb/distance", 1000, distance_Callback);
	ros::Subscriber tablet_sub = n.subscribe("/tablet/info", 1000, tablet_callback);

	image_transport::Publisher imgL_pub = it.advertise("/near/aug/left", 1);
	image_transport::Publisher imgR_pub = it.advertise("/near/aug/right", 1);

	sensor_msgs::Image imgL_;
	sensor_msgs::Image imgR_;

	ros::Rate looprate(100);

	int framePrev = 0;

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

	std::cout << "Camera left:\n";
	std::cout << cameraMatrixL << "\n";
	std::cout << "Camera right:\n";
	std::cout << cameraMatrixR << std::endl;


	cv::Mat t(3,1, CV_64FC1);
	t.setTo(0.0);

	// convert Matrices to KDL

	conversions::rvectvecToKdlFrame(Rstereo, tstereo, Tstereo);
	conversions::rvectvecToKdlFrame(R1, t, R1_kdl);
	conversions::rvectvecToKdlFrame(R2, t, R2_kdl);
	//	std::cout<< "ML: " <<cameraMatrixL <<std::endl;
	//	std::cout<< "DS: " <<distCoeffL <<std::endl;

	char fileNameGauge[256];

//	//load gauge images
//	for(int i = 0; i <=10; i++)
//	{
//		sprintf(fileNameGauge, "%s/gaugeNew/gauge%d.png", path.c_str(), i);
//		//std::cout << "gauge image: " << fileNameGauge << std::endl;
//		gauge[i] = imread(fileNameGauge, -1);
//	}

//	dWidth = 720;
//	dHeight = 576;
	dWidth = imgL.cols;
	dHeight = imgL.rows;

	//opengl
	GLFWwindow* window;

	bufferL_ = new unsigned char[(int)dWidth*(int)dHeight*4];
	bufferR_ = new unsigned char[(int)dWidth*(int)dHeight*4];

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(dWidth * 2, dHeight, "Augmented Reality", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);

	initGL((int)dWidth, (int)dHeight);

	/*
	imgL.create((int)dHeight, (int)dWidth, CV_8UC3);
	imgL.setTo(127);

	imgR.create((int)dHeight, (int)dWidth, CV_8UC3);
	imgR.setTo(63);
	 */


	while (ros::ok() && !glfwWindowShouldClose(window))
	{

		if(!imgL.empty() && !imgR.empty())
		{

			if (tablet_info.buttons[0] && !imgL_rect.empty() && !imgR_rect.empty() )
			{
				// Copy frame to texture
				glBindTexture(GL_TEXTURE_2D, texIdL);
				glTexSubImage2D(
						GL_TEXTURE_2D, 0, 0, 0,
						(int)dWidth, (int)dHeight,
						GL_BGR, GL_UNSIGNED_BYTE, imgL_rect.data);

				// Copy frame to texture
				glBindTexture(GL_TEXTURE_2D, texIdR);
				glTexSubImage2D(
						GL_TEXTURE_2D, 0, 0, 0,
						(int)dWidth, (int)dHeight,
						GL_BGR, GL_UNSIGNED_BYTE, imgR_rect.data);
			}
			else
			{
				// Copy frame to texture
				glBindTexture(GL_TEXTURE_2D, texIdL);
				glTexSubImage2D(
						GL_TEXTURE_2D, 0, 0, 0,
						(int)dWidth, (int)dHeight,
						GL_BGR, GL_UNSIGNED_BYTE, imgL.data);

				// Copy frame to texture
				glBindTexture(GL_TEXTURE_2D, texIdR);
				glTexSubImage2D(
						GL_TEXTURE_2D, 0, 0, 0,
						(int)dWidth, (int)dHeight,
						GL_BGR, GL_UNSIGNED_BYTE, imgR.data);
			}
			/* Do the render */
			render(window);

			/* Poll for and process events */
			glfwPollEvents();

			cv::Mat augL((int)dHeight, (int)dWidth, CV_8UC4, bufferL_);
			cv::Mat augR((int)dHeight, (int)dWidth, CV_8UC4, bufferR_);
			//			std::cout << "cols: " << augL.cols << std::endl;
			//			std::cout << "rows: " << augL.rows << std::endl;

			flip(augL, augL, 0);
			flip(augR, augR, 0);

			if(enableImage)
			{
				cv::putText(augL, "Define Safety Area", cv::Point(60,100), cv::FONT_HERSHEY_PLAIN,
							4.0, cv::Scalar(255,0,0), 3);
			}

			// save images
//			if (frame != framePrev)
//			{
//				char frame_number_char[32];
//				sprintf(frame_number_char, "%06d", frame);
//				cv::imwrite(std::string(CONFIG_FILE_FOLDER) + "images/" + "_Laug_" + frame_number_char + ".jpg", augL);
//			}
			framePrev = frame;

			fromMattoImageA(augL, &imgL_);
			fromMattoImageA(augR, &imgR_);

			//			std::cout << augL << std::endl;

			imgL_pub.publish(imgL_);
			imgR_pub.publish(imgR_);

			//			int count = 0;
			//			for(int i = 0; i < augL.rows; i++)
			//			{
			//				for(int j = 0; j < augL.cols; j++)
			//				{
			//					if(augL.at<cv::Vec4b>(i,j)[3] == 0)
			//						augL.at<cv::Vec4b>(i,j)[3] == 255;
			////						count++;
			//				}
			//			}

			//			std::cout << count << std::endl;
		}
		else
		{

			std::cout << "Cannot read a frame from video stream" << std::endl;

		}

		ros::spinOnce();
		looprate.sleep();
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}

void initGL(int w, int h)
{
	// Background color

	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	//	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Smooth triangles (GL_FLAT for flat triangles)
	glShadeModel(GL_SMOOTH);

	// We don't see the back face
	glCullFace(GL_BACK);
	// The front face is CCW
	glFrontFace(GL_CCW);
	// Disable culling
	glEnable(GL_CULL_FACE);

	// Disable lighting
	glDisable(GL_LIGHTING);

	// Disable depth test
	glEnable(GL_DEPTH_TEST);

	// Enable normalizing
	glEnable(GL_NORMALIZE);

	// Enable blending
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBlendEquation(GL_FUNC_ADD);

	// Generate texture
	glGenTextures(1, &texIdL);
	glBindTexture(GL_TEXTURE_2D, texIdL);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	glGenTextures(1, &texIdR);
	glBindTexture(GL_TEXTURE_2D, texIdR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);

	// Generate gauge textures
//	glGenTextures(11, gaugeTexId);
//	for (int i = 0; i < 11; i++)
//	{
//		glBindTexture(GL_TEXTURE_2D, gaugeTexId[i]);
//		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
//		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
//		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
//		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, gauge[i].cols, gauge[i].rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, gauge[i].data);
//	}
}

void renderSide(
		GLFWwindow* window,
		KDL::Frame &cameraPose,
		cv::Mat &cameraMatrix,
		unsigned char* buffer,
		int x, int width, int height,
		GLuint texId)
{
	// We draw in all the window
	glViewport(x, 0, width, height);

	/* Draw camera image in the background */

	// Set camera
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set 2D camera
	gluOrtho2D(-1.0, 1.0, 1.0, -1.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Disable
	glDisable(GL_DEPTH_TEST);

	// Draw a square with the texture
	if(enableImage)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texId);
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f);
		glVertex3f(1.0f, 1.0f, 0.0f);

		glTexCoord2f(1.0f, 0.0f);
		glVertex3f(1.0f, -1.0f, 0.0f);

		glTexCoord2f(0.0f, 0.0f);
		glVertex3f(-1.0f, -1.0f, 0.0f);

		glTexCoord2f(0.0f, 1.0f);
		glVertex3f(-1.0f, 1.0f, 0.0f);
		glEnd();
	}

	/*
	if (!enableHud)
	{
		glReadPixels(x, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
		return;
	}*/


	if (enableHud)
	{
		// Draw a square with the gauge
		int gaugeIndex = 0;
		float dmin = 0.002;
		float dmax = 0.02;

		if (distance > dmax)
		{
			gaugeIndex = 0;
		}
		else if (distance == -1.0)
		{
			// Warning
			// TODO
		}
		else
		{
			gaugeIndex = (int)(10.0f * (distance - dmax) / (dmin - dmax));
		}

		if (gaugeIndex < 0)
			gaugeIndex = 0;
		else if (gaugeIndex > 10)
			gaugeIndex = 10;

		glEnable(GL_TEXTURE_2D);
//		glBindTexture(GL_TEXTURE_2D, gaugeTexId[gaugeIndex]);
		glColor3f(1.0f, 1.0f, 1.0f);

		glPushMatrix();
		if (x == 0)
			glTranslatef(-0.2f, 0.0f, 0.0f);

		glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 1.0f);
		glVertex3f(1.0f - 0.1f, -0.734f + 0.1f, 0.0f);

		glTexCoord2f(1.0f, 0.0f);
		glVertex3f(1.0f - 0.1f, -1.0f + 0.1f, 0.0f);

		glTexCoord2f(0.0f, 0.0f);
		glVertex3f(0.5f - 0.1f, -1.0f + 0.1f, 0.0f);

		glTexCoord2f(0.0f, 1.0f);
		glVertex3f(0.5f - 0.1f, -0.734f + 0.1f, 0.0f);
		glEnd();

		glPopMatrix();
	}


	// Draw tablet pointer
	if (tablet_info.buttons[0])
	{
		// draw the pointer
		glDisable(GL_TEXTURE_2D);
		glColor3f(0.5f, 1.0f, 0.5f);
		glBegin(GL_LINES);
		glVertex3f(tablet_info.x + 0.03f, tablet_info.y, 0.0f);
		glVertex3f(tablet_info.x - 0.03f, tablet_info.y, 0.0f);
		glEnd();
		glBegin(GL_LINES);
		glVertex3f(tablet_info.x, tablet_info.y + 0.03f, 0.0f);
		glVertex3f(tablet_info.x, tablet_info.y - 0.03f, 0.0f);
		glEnd();

		if(safety_area.size() > 1)
		{
			// draw the contour
			glDisable(GL_TEXTURE_2D);
			glColor3f(0.5f, 1.0f, 0.5f);
			glLineWidth(3.0);
			glBegin(GL_LINES);

			for(int i = 0 ; i < safety_area.size(); i++)
			{
				glVertex3f(safety_area[i].x, safety_area[i].y, 0.0f);
			}

			glEnd();
		}


	}


	/* Draw 3D object */

	if(distance != -1 && enableHud)
	{
		// Set camera
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		// Here set the camera intrinsic parameters
		// fov in degrees, aspect ratio, minimum distance (DO NOT PUT ZERO!), maximum distance

		//	gluPerspective(45.0f, (float)width / (float)height, 0.1f, 1000.0f);

		/*
		float fy = (float)cameraMatrix.at<double>(1,1)/((float)height/2.0);
		float fovy_rad  = 2.0*atan(1.0/fy);
		float fovy_deg = fovy_rad * 180.0 / M_PI;
		gluPerspective(fovy_deg, (float)width / (float)height, 0.001f, 10.0f);*/

		// Calculate perspective matrix
		GLfloat cameraPerspective[16];
		GLfloat zmin = 0.001f;
		GLfloat zmax = 10.0f;

		/*
		cameraPerspective[0] = 2.0f * (float)cameraMatrix.at<double>(0, 0) / (float)width;
		cameraPerspective[1] = 0.0f;
		cameraPerspective[2] = 0.0f;
		cameraPerspective[3] = 0.0f;

		cameraPerspective[4] = 0.0f;
		cameraPerspective[5] = -2.0f * (float)cameraMatrix.at<double>(1, 1) / (float)height;
		cameraPerspective[6] = 0.0f;
		cameraPerspective[7] = 0.0f;

		cameraPerspective[8] = 2.0f * (float)cameraMatrix.at<double>(0, 2) / (float)width - 1.0f;
		cameraPerspective[9] = 1.0f - 2.0f * (float)cameraMatrix.at<double>(1, 2) / (float)height;
		cameraPerspective[10] = -(zmax + zmin) / (zmax - zmin);
		cameraPerspective[11] = 1.0f;

		cameraPerspective[12] = 0.0f;
		cameraPerspective[13] = 0.0f;
		cameraPerspective[14] = 2.0f * zmax * zmin / (zmax - zmin);
		cameraPerspective[15] = 0.0f;*/

		cameraPerspective[0] = 2.0f * (float)cameraMatrix.at<double>(0, 0) / (float)width;
		cameraPerspective[1] = 0.0f;
		cameraPerspective[2] = 0.0f;
		cameraPerspective[3] = 0.0f;

		cameraPerspective[4] = 0.0f;
		cameraPerspective[5] = 2.0f * (float)cameraMatrix.at<double>(1, 1) / (float)height;
		cameraPerspective[6] = 0.0f;
		cameraPerspective[7] = 0.0f;

		cameraPerspective[8] = 2.0f * (float)cameraMatrix.at<double>(0, 2) / (float)width - 1.0f;
		cameraPerspective[9] = 2.0f * (float)cameraMatrix.at<double>(1, 2) / (float)height - 1.0f;
		cameraPerspective[10] = (zmax + zmin) / (zmin - zmax);
		cameraPerspective[11] = -1.0f;

		cameraPerspective[12] = 0.0f;
		cameraPerspective[13] = 0.0f;
		cameraPerspective[14] = 2.0f * zmax * zmin / (zmin - zmax);
		cameraPerspective[15] = 0.0f;

		glMultMatrixf(cameraPerspective);

		//std::cout << "fov: " << fovy_deg << std::endl;
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// Here set the camera INVERSE transform using glMultMatrixf(transform)
		GLfloat camera_pose2[16] = {
				-0.707107, -0.408248, 0.57735, 0.0,
				0.707107, -0.408248, 0.57735, 0.0,
				0.0, 0.816497, 0.57735, 0.0,
				0.0, 0.0, -3.4641, 1.0};
		GLfloat camera_pose[16];

		KDL::Frame cv2gl;
		cv2gl = KDL::Frame::Identity();
		cv2gl.M(0, 0) = 1.0;
		cv2gl.M(1, 1) = -1.0;
		cv2gl.M(2, 2) = -1.0;
		//KDL::Frame cameraPoseInv = (cv2gl * cameraPose ).Inverse();
		KDL::Frame cameraPoseInv = (cameraPose * cv2gl).Inverse();
		//KDL::Frame cameraPoseInv = cv2gl * cameraPose.Inverse();
		//KDL::Frame cameraPoseInv = cameraPose.Inverse() * cv2gl;

		camera_pose[0] = (float)cameraPoseInv.M(0,0);
		camera_pose[1] = (float)cameraPoseInv.M(1,0);
		camera_pose[2] = (float)cameraPoseInv.M(2,0);

		camera_pose[3] = 0.0f;

		camera_pose[4] = (float)cameraPoseInv.M(0,1);
		camera_pose[5] = (float)cameraPoseInv.M(1,1);
		camera_pose[6] = (float)cameraPoseInv.M(2,1);

		camera_pose[7] = 0.0f;

		camera_pose[8] = (float)cameraPoseInv.M(0,2);
		camera_pose[9] = (float)cameraPoseInv.M(1,2);
		camera_pose[10] = (float)cameraPoseInv.M(2,2);

		camera_pose[11] = 0.0f;

		camera_pose[12] = (float)cameraPoseInv.p(0);
		camera_pose[13] = (float)cameraPoseInv.p(1);
		camera_pose[14] = (float)cameraPoseInv.p(2);

		camera_pose[15] = 1.0f;

		glMultMatrixf(camera_pose);

		/*
		std::cout << camera_pose [0] << " , " << camera_pose [1] << " , " <<camera_pose [2] << " , " << camera_pose [3] << " , \n";
		std::cout << camera_pose [4] << " , " << camera_pose [5] << " , " <<camera_pose [6] << " , " << camera_pose [7] << " , \n";
		std::cout << camera_pose [8] << " , " << camera_pose [9] << " , " <<camera_pose [10] << " , " << camera_pose [11] << " , \n";
		std::cout << camera_pose [12] << " , " << camera_pose [13] << " , " <<camera_pose [14] << " , " << camera_pose [15] << std::endl;

		std::cout << "bb:\n";
		std::cout << bb_coord[0] << ", " << bb_coord[1] << "\n";
		std::cout << bb_coord[2] << ", " << bb_coord[3] << "\n";
		std::cout << bb_coord[4] << ", " << bb_coord[5] << std::endl;*/

		// Enable
		glEnable(GL_DEPTH_TEST);

		// Draw a square with the texture
		glDisable(GL_TEXTURE_2D);

		//	std::cout << "bb:\n";
		//	std::cout << bb_coord[0] << ", " << bb_coord[1] << "\n";
		//	std::cout << bb_coord[2] << ", " << bb_coord[3] << "\n";
		//	std::cout << bb_coord[4] << ", " << bb_coord[5] << std::endl;

		/*
		bb_coord[0] = -1.0f;
		bb_coord[1] = 1.0f;
		bb_coord[2] = -1.0f;
		bb_coord[3] = 1.0f;
		bb_coord[4] = -1.0f;
		bb_coord[5] = 1.0f;
		 */


		drawElipsoid(
				bb_coord[0], bb_coord[1],
				bb_coord[2], bb_coord[3],
				bb_coord[4], bb_coord[5]);

		/*drawBoundinBox(	bb_coord[0], bb_coord[1],
				bb_coord[2], bb_coord[3],
				bb_coord[4], bb_coord[5]);*/
	}

	glReadPixels(x, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
}

void render(GLFWwindow* window)
{
	// Get window dimensions
	int width;
	int height;
	glfwGetFramebufferSize(window, &width, &height);

	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderSide(
			window,
			cameraPoseLeft, cameraMatrixL, bufferL_,
			0, width / 2, height, texIdL);

	if (!tablet_info.buttons[0])
	{
		renderSide(
				window,
				cameraPoseRight, cameraMatrixL, bufferR_,
				width / 2, width / 2, height, texIdR);
	}
	else
	{
		renderSide(
				window,
				cameraPoseLeft, cameraMatrixL, bufferR_,
				width / 2, width / 2, height, texIdL);
	}

	// Swap buffers
	glfwSwapBuffers(window);
}