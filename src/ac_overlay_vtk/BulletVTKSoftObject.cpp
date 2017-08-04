//
// Created by nima on 04/08/17.
//

#include <src/ac_overlay_vtk/LoadObjGL/LoadMeshFromObj.h>
#include <ros/ros.h>
#include "BulletVTKSoftObject.h"
#include <sys/stat.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>


inline bool FileExists (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

BulletVTKSoftObject::BulletVTKSoftObject(btSoftBodyWorldInfo &world_info,
                                         const std::string mesh_file_dir,
                                         float *pose,
                                         float density,
                                         float friction) {



    double volume = 0.0;
    std::string shape_string; // for debug report



//    if(!FileExists(mesh_file_dir.c_str())) {
//        ROS_ERROR("Can't open mesh file: %s", mesh_file_dir.c_str());
//        throw std::runtime_error("Can't open mesh file.");
//    }
//    else
//        ROS_DEBUG("Loading mesh file from: %s", mesh_file_dir.c_str()) ;
//
//    collision_shape_ = LoadCompoundMeshFromObj(mesh_file_dir, B_DIM_SCALE);
//
//    // set name
//    shape_string = collision_shape_->getName();;


    body_ = btSoftBodyHelpers::CreateEllipsoid(
            world_info,
            btVector3(pose[0]*B_DIM_SCALE,
                      pose[1]*B_DIM_SCALE,
                      pose[2]*B_DIM_SCALE),
            btVector3(0.01f * B_DIM_SCALE,
                      0.01f * B_DIM_SCALE,
                      0.01f * B_DIM_SCALE)
            ,1000);
    body_->m_cfg.viterations=20;
    body_->m_cfg.piterations=20;
    body_->m_cfg.kPR = 100 /B_DIM_SCALE;
    body_->setTotalDensity(density/(B_DIM_SCALE*B_DIM_SCALE));
//    sb->setMass(0, 0);
    body_->getCollisionShape()->setMargin(0.08);
//    sb->generateBendingConstraints(3);





    // Create a polydata object
    vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();

    mapper->SetInputData(polyData);
    actor_ = vtkSmartPointer<vtkActor>::New();
    actor_->SetMapper(mapper);

}


//------------------------------------------------------------------------------

void BulletVTKSoftObject::RenderSoftbody() {
    // first attempt. I doubt it is the most efficient way.
    //faces
    vtkSmartPointer<vtkCellArray> triangles =
            vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    // Create a polydata object
    vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

    std::vector<vtkSmartPointer<vtkTriangle> >triangle;

    for(int i=0;i<body_->m_faces.size();i++)
    {
        triangle.push_back(vtkSmartPointer<vtkTriangle>::New());

        for(int j=0;j<3;j++) {
            points->InsertNextPoint(body_->m_faces[i].m_n[j]->m_x.x()/B_DIM_SCALE,
                                    body_->m_faces[i].m_n[j]->m_x.y()
                                    /B_DIM_SCALE,
                                    body_->m_faces[i].m_n[j]->m_x.z()/B_DIM_SCALE);

            triangle[i]->GetPointIds()->SetId ( j, i*3  + j );
        }

        triangles->InsertNextCell ( triangle[i] );
    }
    // Add the geometry and topology to the polydata
    polyData->SetPoints ( points );
    polyData->SetPolys ( triangles );
    mapper->SetInputData(polyData);
    actor_->SetMapper(mapper);
////    for(int i=0;i<b->m_links.size();i++)
//    {
//        for(int j=0;j<2;j++)
//            (b->m_links[i].m_n[j]->m_x.x(),
//                       b->m_links[i].m_n[j]->m_x.y(),
//                       b->m_links[i].m_n[j]->m_x.z());
//    }

}