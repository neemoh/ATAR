#include "LoadMeshFromObj.h"

#include "GLInstanceGraphicsShape.h"
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <iostream>
#include "Bullet3Common/b3HashMap.h"
#include "Wavefront2GLInstanceGraphicsShape.h"

struct CachedObjResult
{
    std::string m_msg;
    std::vector<tinyobj::shape_t> m_shapes;
};

static b3HashMap<b3HashString, CachedObjResult> gCachedObjResults;
static int gEnableFileCaching = 1;

void b3EnableFileCaching(int enable)
{
    gEnableFileCaching  = enable;
    if (enable==0)
    {
        gCachedObjResults.clear();
    }
}


std::string LoadFromCachedOrFromObj(
        std::vector<tinyobj::shape_t>& shapes,   // [output]
        const char* filename,
        const char* mtl_basepath)
{
    CachedObjResult* resultPtr = gCachedObjResults[filename];
    if (resultPtr)
    {
        const CachedObjResult& result = *resultPtr;
        shapes = result.m_shapes;
        return result.m_msg;
    }

    std::string err = tinyobj::LoadObj(shapes, filename, mtl_basepath);
    CachedObjResult result;
    result.m_msg = err;
    result.m_shapes = shapes;
    if (gEnableFileCaching)
    {
        gCachedObjResults.insert(filename,result);
    }
    return err;
}


GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath)
{
    B3_PROFILE("LoadMeshFromObj");
    std::vector<tinyobj::shape_t> shapes;
    {
        B3_PROFILE("tinyobj::LoadObj2");
        std::string  err  = LoadFromCachedOrFromObj(shapes, relativeFileName, materialPrefixPath);
    }

    {
        B3_PROFILE("btgCreateGraphicsShapeFromWavefrontObj");
        GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
        return gfxShape;
    }
}



btCompoundShape* LoadMeshFromObjs(const char* relativeFileName, const
char* materialPrefixPath)
{
    B3_PROFILE("LoadMeshFromObj");
    std::vector<tinyobj::shape_t> shapes;
    {
        B3_PROFILE("tinyobj::LoadObj2");
        std::string  err  = LoadFromCachedOrFromObj(shapes, relativeFileName, materialPrefixPath);
    }

    {
        btCompoundShape* compound = new btCompoundShape();

        for (int i = 0; i < shapes.size(); ++i) {
            std::vector<tinyobj::shape_t> shape = {shapes[i]};

            GLInstanceGraphicsShape *gfxShape = btgCreateGraphicsShapeFromWavefrontObj(
                    shape);

            // create convex hull
            const GLInstanceVertex &v = gfxShape->m_vertices->at(0);
            btConvexHullShape *btCHshape = new btConvexHullShape(
                    (const btScalar *) (&(v.xyzw[0])),
                    gfxShape->m_numvertices,
                    sizeof(GLInstanceVertex));
            btCHshape->initializePolyhedralFeatures();
            //calculate the centroid
            btVector3 centroid;
            for (int j = 0; j <gfxShape->m_numvertices; ++j) {
                centroid += btVector3(gfxShape->m_vertices->at(0).xyzw[0],
                                      gfxShape->m_vertices->at(0).xyzw[1],
                                      gfxShape->m_vertices->at(0).xyzw[2]);
            }
            centroid *= 1.f/(float(gfxShape->m_numvertices) );
            std::cout << "centroid: " << centroid << std::endl;
            // add to the compound shape
            btTransform trans;
            trans.setIdentity();
            trans.setOrigin(centroid);
            compound->addChildShape(trans, btCHshape);
        }

        return compound;
    }
}
