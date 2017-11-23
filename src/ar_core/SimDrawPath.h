//
// Created by nima on 23/11/17.
//

#ifndef ATAR_SIMDRAWPATH_H
#define ATAR_SIMDRAWPATH_H


#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <kdl/frames.hpp>

class SimDrawPath {
public:

    SimDrawPath();

    void InsertNewPoint(KDL::Vector in);

    vtkSmartPointer<vtkActor> GetActor() { return path_actor; };

    void Clear();

private:
    bool initialized;
    vtkSmartPointer<vtkPoints> path_points = vtkPoints::New();
    vtkSmartPointer<vtkCellArray> path_cell = vtkCellArray::New();
    vtkSmartPointer<vtkPolyData> path_poly = vtkPolyData::New();
    vtkSmartPointer<vtkActor> path_actor = vtkActor::New();

};


#endif //ATAR_SIMDRAWPATH_H
