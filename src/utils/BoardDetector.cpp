/*
 * BoardDetector.cpp
 *
 *  Created on: Dec 13, 2016
 *      Author: nima
 */

#include <ros/ros.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <utils/Colors.hpp>
#include "utils/BoardDetector.hpp"

//-----------------------------------------------------------------------------------
// BOARD DETECTOR CLASS METHODS
//-----------------------------------------------------------------------------------

BoardDetector::BoardDetector(ArucoBoard board, CameraDistortion camera, double n_avg)
: Board(board),
  n_avg(n_avg),
  Image(),
  _camera(camera)
{
	_aruco.DetectorParams = cv::aruco::DetectorParameters::create();
	_aruco.DetectorParams->doCornerRefinement = true;

	_aruco.Dictionary = cv::aruco::getPredefinedDictionary(
			cv::aruco::PREDEFINED_DICTIONARY_NAME(Board.DictionaryID));

	_aruco.Gridboard = cv::aruco::GridBoard::create(
			Board.Height, Board.Width, Board.MarkerLength, Board.MarkerSeparation,
			_aruco.Dictionary
	);
	_aruco.Board = _aruco.Gridboard.staticCast<cv::aruco::Board>();
}


void BoardDetector::Detect() {

	cv::Vec3d rotation_current, translation_current;

	if (Image.empty())
		ROS_ERROR("Empty image received. Is an image source present?");

	// detect markers
	cv::aruco::detectMarkers(
			Image, _aruco.Dictionary, _aruco.DetectedCorners, _aruco.DetectedMarkerIds,
			_aruco.DetectorParams, _aruco.RejectedCorners
	);

	// Should we try to find the other markers ?
	if (_aruco.RefindStrategy)
		cv::aruco::refineDetectedMarkers(
				Image, _aruco.Board, _aruco.DetectedCorners, _aruco.DetectedMarkerIds,
				_aruco.RejectedCorners, _camera.camMatrix, _camera.distCoeffs
		);

	// if we have found some markers on the image we can estimate the pose
	if (_aruco.DetectedMarkerIds.size() > 0) {

		try{
			auto num_markers_used = cv::aruco::estimatePoseBoard(
					_aruco.DetectedCorners, _aruco.DetectedMarkerIds, _aruco.Board,
					_camera.camMatrix, _camera.distCoeffs,
					rotation_current, translation_current
			);
			_pose_estimated = (num_markers_used > 0);
		}
		catch (const cv::Exception& e){
			ROS_ERROR("Something went wrong in cv::aruco::estimatePoseBoard");
		}

		// average approximation of the orientation to fix the oscillation of the axes
		rvec -= rvec / n_avg;
		rvec += rotation_current / n_avg;

		// averaging the origin position
		tvec -= tvec / n_avg;
		tvec += translation_current / n_avg;

		// wait for the initialization of the averaging (4 times the avg window should be enough)
		if (frame_counter < n_avg * 4)
			frame_counter++;
		else
			_ready = true;
	} else
		_ready = false;

}


void BoardDetector::DrawAxis() {
	if (_pose_estimated)
		drawAxisAntiAliased(Image, _camera.camMatrix, _camera.distCoeffs, rvec, tvec, 0.01);
}


void BoardDetector::drawAxisAntiAliased(
		const cv::_InputOutputArray &_image, const cv::_InputArray &_cameraMatrix,
		const cv::_InputArray &_distCoeffs, const cv::_InputArray &_rvec, const cv::_InputArray &_tvec,
		float length)
{
	CV_Assert(_image.getMat().total() != 0 &&
			(_image.getMat().channels() == 1 || _image.getMat().channels() == 3));
	CV_Assert(length > 0);

	// project axis points
	std::vector<cv::Point3f> axisPoints {
		cv::Point3f( 0,  0,  0),
				cv::Point3f( 1,  0,  0) * length,
				cv::Point3f( 0,  1,  0) * length,
				cv::Point3f( 0,  0,  1) * length
	};

	std::vector<cv::Point2f> imagePoints;
	cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

	// draw axis lines
	cv::line(_image, imagePoints[0], imagePoints[1], Colors::Blue, 2, CV_AA);
	cv::line(_image, imagePoints[0], imagePoints[2], Colors::Green, 2, CV_AA);
	cv::line(_image, imagePoints[0], imagePoints[3], Colors::Red, 2, CV_AA);
}


void BoardDetector::DrawDetectedMarkers() {
	if (_aruco.DetectedMarkerIds.size() > 0)
		cv::aruco::drawDetectedMarkers(Image, _aruco.DetectedCorners, _aruco.DetectedMarkerIds);
}


void BoardDetector::PushImage(cv::Mat &image) {
	Image = image;

	if (image.empty()) {
		throw std::runtime_error("Error: BoardDetector::PushImage received an empty image.");
	}

	Detect();

	if (_ready) {
		DrawAxis();
		// DrawDetectedMarkers();

	}
}
