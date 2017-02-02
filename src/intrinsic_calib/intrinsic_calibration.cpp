#include <iostream>
#include <sstream>
#include <stdio.h>


#define WITH_ROS

#ifdef WITH_ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;
#define WITH_ROS
cv::Mat image;

static void help() {
    cout << "This is a camera calibration sample." << endl
         << "Usage: calibration configurationFile" << endl
         << "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}

class Settings {
public:
    Settings() : goodInput(false) {
    }

    enum class Pattern {
        NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID
    };
    enum class InputType {
        INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST, ROS_TOPIC
    };

    void write(FileStorage &fs) const                        //Write serialization for this class
    {
        fs << "{"
           << "BoardSize_Width" << boardSize.width
           << "BoardSize_Height" << boardSize.height
           << "Square_Size" << squareSize
           << "Calibrate_Pattern" << patternToUse
           << "Calibrate_NrOfFrameToUse" << nrFrames
           << "Calibrate_FixAspectRatio" << aspectRatio
           << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
           << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

           << "Write_DetectedFeaturePoints" << writePoints
           << "Write_extrinsicParameters" << writeExtrinsics
           << "Write_outputFileName" << outputFileName

           << "Show_UndistortedImage" << showUndistorsed

           << "Input_FlipAroundHorizontalAxis" << flipVertical
           << "Input_Delay" << delay
           << "Input" << input
           << "}";
    }

    void read(const FileNode &node)                          //Read serialization for this class
    {
        node["BoardSize_Width"] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"] >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> writePoints;
        node["Write_extrinsicParameters"] >> writeExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Calibrate_UseFisheyeModel"] >> useFisheye;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;

        validate();
    }

    void validate() {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0) {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6) {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0) {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
            inputType = InputType::INVALID;
        else {
            if (input[0] >= '0' && input[0] <= '9') {
                stringstream ss(input);
                ss >> cameraID;
                inputType = InputType::CAMERA;
            } else {
                if (input.substr(0, 6) == "ros://") {
                    inputType = InputType::ROS_TOPIC;
                    ros_topic = input.substr(6, string::npos);
#ifndef WITH_ROS
                    cerr << "The input type is a ROS topic but this program has not been compiled "
                         << "with ROS. To enabled ros pass '-DWITH_ROS=ON' to CMake or add "
                         << "'#define WITH_ROS' to the top of 'opencv_cam_calib.cpp'" << endl;
#endif

                } else if (readStringList(input, imageList)) {
                    inputType = InputType::IMAGE_LIST;
                    nrFrames = (nrFrames < (int) imageList.size()) ? nrFrames
                                                                   : (int) imageList.size();
                } else
                    inputType = InputType::VIDEO_FILE;
            }
            if (inputType == InputType::CAMERA)
                inputCapture.open(cameraID);
            if (inputType == InputType::VIDEO_FILE)
                inputCapture.open(input);
#ifdef WITH_ROS
            if (inputType == InputType::ROS_TOPIC) {
                auto img = ros::topic::waitForMessage<sensor_msgs::Image>(ros_topic, ros::Duration(1));
                if (!img) {
                    cerr << "Warning: ROS topic does not seem to be publishing any frames" << endl;
                    inputType = InputType::INVALID;

                }
            }
#endif
            if ((inputType == InputType::CAMERA || inputType == InputType::VIDEO_FILE) &&
                (!inputCapture.isOpened())) {
                inputType = InputType::INVALID;
            }
        }
        if (inputType == InputType::INVALID) {
            cerr << " Input does not exist: " << input << endl;
            goodInput = false;
        }

        flag = CALIB_FIX_K4 | CALIB_FIX_K5;
        if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if (calibZeroTangentDist) flag |= CALIB_ZERO_TANGENT_DIST;
        if (aspectRatio) flag |= CALIB_FIX_ASPECT_RATIO;

        if (useFisheye) {
            // the fisheye model has its own enum, so overwrite the flags
            flag = fisheye::CALIB_FIX_SKEW | fisheye::CALIB_RECOMPUTE_EXTRINSIC |
                   // fisheye::CALIB_FIX_K1 |
                   fisheye::CALIB_FIX_K2 | fisheye::CALIB_FIX_K3 | fisheye::CALIB_FIX_K4;
        }

        calibrationPattern = Pattern::NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = Pattern::CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = Pattern::CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID"))
            calibrationPattern = Pattern::ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == Pattern::NOT_EXISTING) {
            cerr << " Camera calibration mode does not exist: " << patternToUse << endl;
            goodInput = false;
        }
        atImageList = 0;

    }


    Mat nextImage() {
#ifdef WITH_ROS
        if (inputType == InputType::ROS_TOPIC) {
            // Wait until we actually have a non empty image before returning
            ros::Rate loop_rate(24);
            while (true) {
                if (!image.empty()) return image;

                ros::spinOnce();
                loop_rate.sleep();
            }

        }
#endif
        Mat result;
        if (inputCapture.isOpened()) {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        } else if (atImageList < imageList.size())
            result = imread(imageList[atImageList++], IMREAD_COLOR);

        return result;
    }

    static bool readStringList(const string &filename, vector<string> &l) {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if (!fs.isOpened())
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if (n.type() != FileNode::SEQ)
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for (; it != it_end; ++it)
            l.push_back((string) * it);
        return true;
    }

public:
    Size boardSize;              // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;  // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;            // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool writePoints;            // Write detected feature points
    bool writeExtrinsics;        // Write extrinsic parameters
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorsed;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration

    int cameraID;
    vector<string> imageList;
    string ros_topic;

    size_t atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;

};

static inline void
read(const FileNode &node, Settings &x, const Settings &default_value = Settings()) {
    if (node.empty())
        x = default_value;
    else
        x.read(node);
}

static inline void write(FileStorage &fs, const String &, const Settings &s) {
    s.write(fs);
}


#ifdef WITH_ROS
void OnNewCameraImage(const sensor_msgs::ImageConstPtr& msg) {
    try {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}
#endif

enum class State {
    DETECTION = 0, CAPTURING = 1, CALIBRATED = 2
};

bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
    vector<vector<Point2f> > imagePoints);

int main(int argc, char *argv[]) {
    help();

#ifdef WITH_ROS
    ros::init(argc, argv, "camera_calibration");
    ros::NodeHandle node;
#endif

    //! [file_read]
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened()) {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file
    //! [file_read]

    //FileStorage fout("settings.yml", FileStorage::WRITE); // write config as YAML
    //fout << "Settings" << s;

    if (!s.goodInput) {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

#ifdef WITH_ROS
    ros::Subscriber img_source = node.subscribe(s.ros_topic, 1, OnNewCameraImage);
#endif

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    State mode = (s.inputType == Settings::InputType::IMAGE_LIST) ?
                 State::CAPTURING : State::DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0, 0, 255), GREEN(0, 255, 0);
    const char ESC_KEY = 27;

#ifdef WITH_ROS
    ros::Rate rate(24);
#endif

    //! [get_input]
    for (;;) {
#ifdef WITH_ROS
        ros::spinOnce();
        rate.sleep();
#endif

        Mat view;
        bool blinkOutput = false;

        view = s.nextImage();

        //-----  If no more image, or got enough, then stop calibration and show result -------------
        if (mode == State::CAPTURING && imagePoints.size() >= (size_t) s.nrFrames) {
            if (runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints))
                mode = State::CALIBRATED;
            else
                mode = State::DETECTION;
        }

        if (view.empty())          // If there are no more images stop the loop
        {
            // if calibration threshold was not reached yet, calibrate now
            if (mode != State::CALIBRATED && !imagePoints.empty())
                runCalibrationAndSave(s, imageSize, cameraMatrix, distCoeffs, imagePoints);
            break;
        }
        //! [get_input]

        imageSize = view.size();  // Format input image.
        if (s.flipVertical) flip(view, view, 0);

        //! [find_pattern]
        vector<Point2f> pointBuf;

        bool found;

        int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

        if (!s.useFisheye) {
            // fast check erroneously fails with high distortions like fisheye
            chessBoardFlags |= CALIB_CB_FAST_CHECK;
        }

        switch (s.calibrationPattern) // Find feature points on the input format
        {
            case Settings::Pattern::CHESSBOARD:
                found = findChessboardCorners(view, s.boardSize, pointBuf, chessBoardFlags);
                break;
            case Settings::Pattern::CIRCLES_GRID:
                found = findCirclesGrid(view, s.boardSize, pointBuf);
                break;
            case Settings::Pattern::ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid(view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID);
                break;
            default:
                found = false;
                break;
        }
        //! [find_pattern]
        //! [pattern_found]
        if (found)                // If done with success,
        {
            // improve the found corners' coordinate accuracy for chessboard
            if (s.calibrationPattern == Settings::Pattern::CHESSBOARD) {
                Mat viewGray;
                cvtColor(view, viewGray, COLOR_BGR2GRAY);
                cornerSubPix(viewGray, pointBuf, Size(11, 11),
                    Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            }

            if (mode == State::CAPTURING &&  // For camera only take new samples after delay time
                (!s.inputCapture.isOpened() ||
                 clock() - prevTimestamp > s.delay * 1e-3 * CLOCKS_PER_SEC)) {
                imagePoints.push_back(pointBuf);
                prevTimestamp = clock();
                blinkOutput = s.inputCapture.isOpened();
            }

            // Draw the corners.
            drawChessboardCorners(view, s.boardSize, Mat(pointBuf), found);
        }
        //! [pattern_found]
        //----------------------------- Output Text ------------------------------------------------
        //! [output_text]
        string msg = (mode == State::CAPTURING) ? "100/100" :
                     mode == State::CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);

        if (mode == State::CAPTURING) {
            if (s.showUndistorsed)
                msg = format("%d/%d Undist", (int) imagePoints.size(), s.nrFrames);
            else
                msg = format("%d/%d", (int) imagePoints.size(), s.nrFrames);
        }

        putText(view, msg, textOrigin, 1, 1, mode == State::CALIBRATED ? GREEN : RED);

        if (blinkOutput)
            bitwise_not(view, view);
        //! [output_text]
        //------------------------- Video capture  output  undistorted ------------------------------
        //! [output_undistorted]
        if (mode == State::CALIBRATED && s.showUndistorsed) {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }
        //! [output_undistorted]
        //------------------------------ Show image and check for input commands -------------------
        //! [await_input]
        imshow("Image View", view);
        char key = (char) waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if (key == ESC_KEY)
            break;

        if (key == 'u' && mode == State::CALIBRATED)
            s.showUndistorsed = !s.showUndistorsed;

        // If the input is a ros topic then we should know by now that the topic exists and is
        // publishing (otherwise nextImage would hang). If on the hand it is a Camera or Video
        // topic then we need to check whether it was actually opened.
        if (key == 'g'
            && ((s.inputType == Settings::InputType::ROS_TOPIC)
                || (s.inputType != Settings::InputType::ROS_TOPIC && s.inputCapture.isOpened())))
        {
            mode = State::CAPTURING;
            imagePoints.clear();
        }
        //! [await_input]
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    //! [show_results]
    if (s.inputType == Settings::InputType::IMAGE_LIST && s.showUndistorsed) {
        Mat view, rview, map1, map2;

        if (s.useFisheye) {
            Mat newCamMat;
            fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                Matx33d::eye(), newCamMat, 1);
            fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat,
                imageSize,
                CV_16SC2, map1, map2);
        } else {
            initUndistortRectifyMap(
                cameraMatrix, distCoeffs, Mat(),
                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                imageSize,
                CV_16SC2, map1, map2);
        }

        for (size_t i = 0; i < s.imageList.size(); i++) {
            view = imread(s.imageList[i], 1);
            if (view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char) waitKey();
            if (c == ESC_KEY || c == 'q' || c == 'Q')
                break;
        }
    }
    //! [show_results]

    return 0;
}

//! [compute_errors]
static double computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints,
    const vector<vector<Point2f> > &imagePoints,
    const vector<Mat> &rvecs, const vector<Mat> &tvecs,
    const Mat &cameraMatrix, const Mat &distCoeffs,
    vector<float> &perViewErrors, bool fisheye) {
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        if (fisheye) {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix,
                distCoeffs);
        } else {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs,
                imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

//! [compute_errors]
//! [board_corners]
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f> &corners,
    Settings::Pattern patternType /*= Settings::CHESSBOARD*/) {
    corners.clear();

    switch (patternType) {
        case Settings::Pattern::CHESSBOARD:
        case Settings::Pattern::CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; ++i)
                for (int j = 0; j < boardSize.width; ++j)
                    corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
            break;

        case Settings::Pattern::ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(Point3f((2 * j + i % 2) * squareSize, i * squareSize, 0));
            break;
        default:
            break;
    }
}

//! [board_corners]
static bool runCalibration(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
    vector<vector<Point2f> > imagePoints, vector<Mat> &rvecs, vector<Mat> &tvecs,
    vector<float> &reprojErrs, double &totalAvgErr) {
    //! [fixed_aspect]
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0, 0) = s.aspectRatio;
    //! [fixed_aspect]
    if (s.useFisheye) {
        distCoeffs = Mat::zeros(4, 1, CV_64F);
    } else {
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms;

    if (s.useFisheye) {
        Mat _rvecs, _tvecs;
        rms = fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs,
            _rvecs,
            _tvecs, s.flag);

        rvecs.reserve(_rvecs.rows);
        tvecs.reserve(_tvecs.rows);
        for (int i = 0; i < int(objectPoints.size()); i++) {
            rvecs.push_back(_rvecs.row(i));
            tvecs.push_back(_tvecs.row(i));
        }
    } else {
        rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs,
            tvecs,
            s.flag);
    }

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix,
        distCoeffs, reprojErrs, s.useFisheye);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(Settings &s, Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs,
    const vector<Mat> &rvecs, const vector<Mat> &tvecs,
    const vector<float> &reprojErrs, const vector<vector<Point2f> > &imagePoints,
    double totalAvgErr) {
    FileStorage fs(s.outputFileName, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf), "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nr_of_frames" << (int) std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << s.boardSize.width;
    fs << "board_height" << s.boardSize.height;
    fs << "square_size" << s.squareSize;

    if (s.flag & CALIB_FIX_ASPECT_RATIO)
        fs << "fix_aspect_ratio" << s.aspectRatio;

    if (s.flag) {
        if (s.useFisheye) {
            sprintf(buf, "flags:%s%s%s%s%s%s",
                s.flag & fisheye::CALIB_FIX_SKEW ? " +fix_skew" : "",
                s.flag & fisheye::CALIB_FIX_K1 ? " +fix_k1" : "",
                s.flag & fisheye::CALIB_FIX_K2 ? " +fix_k2" : "",
                s.flag & fisheye::CALIB_FIX_K3 ? " +fix_k3" : "",
                s.flag & fisheye::CALIB_FIX_K4 ? " +fix_k4" : "",
                s.flag & fisheye::CALIB_RECOMPUTE_EXTRINSIC ? " +recompute_extrinsic" : "");
        } else {
            sprintf(buf, "flags:%s%s%s%s",
                s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
                s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
                s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
                s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
        }
        cvWriteComment(*fs, buf, 0);
    }

    fs << "flags" << s.flag;

    fs << "fisheye_model" << s.useFisheye;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (s.writeExtrinsics && !reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (s.writeExtrinsics && !rvecs.empty() && !tvecs.empty()) {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
        for (size_t i = 0; i < rvecs.size(); i++) {
            Mat r = bigmat(Range(int(i), int(i + 1)), Range(0, 3));
            Mat t = bigmat(Range(int(i), int(i + 1)), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if (s.writePoints && !imagePoints.empty()) {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (size_t i = 0; i < imagePoints.size(); i++) {
            Mat r = imagePtMat.row(int(i)).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

//! [run_and_save]
bool runCalibrationAndSave(Settings &s, Size imageSize, Mat &cameraMatrix, Mat &distCoeffs,
    vector<vector<Point2f> > imagePoints) {
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
        reprojErrs,
        totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
         << ". avg re projection error = " << totalAvgErr << endl;

    if (ok)
        saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, reprojErrs,
            imagePoints,
            totalAvgErr);
    return ok;
}
//! [run_and_save]

