//
// Created by nima on 12/10/17.
//

#ifndef ATAR_VTKCONVERSIONS_H
#define ATAR_VTKCONVERSIONS_H

#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>
#include <kdl/frames.hpp>
#include "opencv2/opencv.hpp"

namespace VTKConversions{

    void AxisAngleToVTKMatrix (const cv::Vec3d cam_rvec, const cv::Vec3d cam_tvec,
                               vtkSmartPointer<vtkMatrix4x4> out);

    void KDLFrameToVTKMatrix (const KDL::Frame in,
                               vtkSmartPointer<vtkMatrix4x4> out);

    void VTKMatrixToKDLFrame(const vtkSmartPointer<vtkMatrix4x4> in,
                                               KDL::Frame  & out);

}


#endif