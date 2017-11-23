//
// Created by nima on 23/11/17.
//

#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include "SimDrawPath.h"


SimDrawPath::SimDrawPath() {

    //    First we'll create an initial point to build on
    path_points->InsertPoint(0, 0.0, 0.0, 0.0);
    path_cell->InsertNextCell(1) ;
    path_cell->InsertCellPoint(0);

    path_poly->SetPoints(path_points);
    path_poly->SetLines(path_cell);

    vtkSmartPointer<vtkPolyDataMapper> path_mapper = vtkPolyDataMapper::New();
    path_mapper->SetInputData(path_poly);

    path_actor->SetMapper(path_mapper);

}


void SimDrawPath::InsertNewPoint(KDL::Vector in) {

    if(!initialized){
        path_points->InsertPoint(0,  in[0], in[1], in[2]);
        path_cell->InsertNextCell(1) ;
        path_cell->InsertCellPoint(0);
        initialized = true;
    }
    else
    {
        auto num_points = path_points->GetNumberOfPoints();
        path_points->InsertPoint(num_points, in[0], in[1], in[2]);
        path_points->Modified();

        path_cell->InsertNextCell(2);
        path_cell->InsertCellPoint(num_points - 1);
        path_cell->InsertCellPoint(num_points);
        path_cell->Modified();
    }

}

void SimDrawPath::Clear() {

    path_points->Reset();
    path_cell->Reset();
    path_poly->Reset();
    initialized=false;

}
