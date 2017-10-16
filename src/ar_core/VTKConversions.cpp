//
// Created by nima on 12/10/17.
//

#include "VTKConversions.h"



//------------------------------------------------------------------------------
void VTKConversions::AxisAngleToVTKMatrix(const cv::Vec3d cam_rvec,
                                          const cv::Vec3d cam_tvec,
                                          vtkSmartPointer<vtkMatrix4x4>  out) {


    cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
    cv::Rodrigues(cam_rvec, rotationMatrix);

    out->Identity();

    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out->SetElement(i, j, rotationMatrix.at<double>(i, j));
        }
        out->SetElement(i, 3, cam_tvec[i]);
    }


}


//------------------------------------------------------------------------------
void ::VTKConversions::KDLFrameToVTKMatrix(const KDL::Frame in,
                                           vtkSmartPointer<vtkMatrix4x4> out) {
    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out->SetElement(i, j, in.M(i,j));
        }
        out->SetElement(i, 3, in.p[i]);
    }

}


//------------------------------------------------------------------------------
void VTKConversions::VTKMatrixToKDLFrame(const vtkSmartPointer<vtkMatrix4x4> in,
                                         KDL::Frame & out) {
    // Convert to VTK matrix.
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out.M(i, j) = in->GetElement(i, j);
        }
        out.p[i] = in->GetElement(i, 3);
    }

}