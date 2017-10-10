//
// Created by nima on 04/08/17.
//

#include <src/ar_core/LoadObjGL/LoadMeshFromObj.h>
#include <ros/ros.h>
#include "SimSoftObject.h"
#include <sys/stat.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <vtkCellArray.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <src/ar_core/LoadObjGL/GLInstanceGraphicsShape.h>


inline bool FileExists (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

SimSoftObject::SimSoftObject(btSoftBodyWorldInfo &world_info,
                                         const std::string mesh_file_dir,
                                         KDL::Frame pose,
                                         float density,
                                         float friction) {

    std::string shape_string; // for debug report

    if(!FileExists(mesh_file_dir.c_str())) {
        ROS_ERROR("Can't open mesh file: %s", mesh_file_dir.c_str());
        throw std::runtime_error("Can't open mesh file.");
    }
    else
        ROS_DEBUG("Loading mesh file from: %s", mesh_file_dir.c_str()) ;

    collision_shape_ = LoadCompoundMeshFromObj(mesh_file_dir, B_DIM_SCALE);

    // set name
    shape_string = collision_shape_->getName();;

//    const btVector3	h=s*0.5;
//    const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
//                                p+h*btVector3(+1,-1,-1),
//                                p+h*btVector3(-1,+1,-1),
//                                p+h*btVector3(+1,+1,-1),
//                                p+h*btVector3(-1,-1,+1),
//                                p+h*btVector3(+1,-1,+1),
//                                p+h*btVector3(-1,+1,+1),
//                                p+h*btVector3(+1,+1,+1)};
//    btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
//    psb->generateBendingConstraints(2);

    GLInstanceGraphicsShape* gfxShape = LoadMeshFromObj(mesh_file_dir);

    btVector3 v[gfxShape->m_numvertices];
    for (int i = 0; i < gfxShape->m_numvertices; ++i) {
        v[i] = btVector3(gfxShape->m_vertices->at(i).xyzw[0]*B_DIM_SCALE,
                         gfxShape->m_vertices->at(i).xyzw[1]*B_DIM_SCALE,
                         gfxShape->m_vertices->at(i).xyzw[2]*B_DIM_SCALE);
    }
    body_=btSoftBodyHelpers::CreateFromConvexHull(world_info,
                                                  v,
                                                  gfxShape->m_numvertices);

//    btSoftBody::Material*	pm=body_->appendMaterial();
//    pm->m_kLST				=	0.5;
//    pm->m_flags				-=	btSoftBody::fMaterial::DebugDraw;
//    body_->generateBendingConstraints(2,pm);
    body_->generateBendingConstraints(2);
    body_->m_cfg.piterations	=	2;
    body_->m_cfg.kDF			=	0.9;
    body_->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
    body_->randomizeConstraints();

    btMatrix3x3	m;
    m.setEulerZYX(1.f, 0.f, 0.f);
    body_->transform(btTransform(m, btVector3((float)pose.p[0]*B_DIM_SCALE,
                                              (float)pose.p[1]*B_DIM_SCALE,
                                              (float)pose.p[2]*B_DIM_SCALE)));
    body_->setTotalMass(0.1);

//    body_->m_cfg.viterations=20;
//    body_->m_cfg.piterations=20;
//    body_->m_cfg.kPR = 100 /B_DIM_SCALE;
//    body_->setTotalDensity(density/(B_DIM_SCALE*B_DIM_SCALE));
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

void SimSoftObject::RenderSoftbody() {
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