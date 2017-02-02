//
// Created by nearlab on 14/12/16.
//

#ifndef TELEOP_VISION_CONVERSIONS_HPP_HPP
#define TELEOP_VISION_CONVERSIONS_HPP_HPP

#include <vector>

#include <opencv2/core.hpp>
#include <geometry_msgs/Pose.h>
#include "kdl/frames.hpp"


namespace conversions {
    // some self explanatory conversions!

    void rvecTokdlRot(const cv::Vec3d _rvec, KDL::Rotation &_kdl);

    void rvectvecToKdlFrame(const cv::Vec3d _rvec, const cv::Vec3d _tvec, KDL::Frame &_kdl);

    void kdlFrameToRvectvec(const KDL::Frame _kdl, cv::Vec3d &_rvec, cv::Vec3d &_tvec);

    void matx33dToKdlRot(const cv::Matx33d _mat, KDL::Rotation &_kdl);

    void kdlRotToMatx33d(const KDL::Rotation _kdl, cv::Matx33d &_mat);

    void poseMsgToVector(const geometry_msgs::Pose in_pose, std::vector<double> &out_vec);

    void vectorToPoseMsg(const std::vector<double> in_vec, geometry_msgs::Pose &out_pose);
};

#endif //TELEOP_VISION_CONVERSIONS_HPP_HPP
