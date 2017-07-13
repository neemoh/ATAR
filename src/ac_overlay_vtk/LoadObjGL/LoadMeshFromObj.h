#ifndef LOAD_MESH_FROM_OBJ_H
#define LOAD_MESH_FROM_OBJ_H


struct GLInstanceGraphicsShape;

#include"tiny_obj_loader.h"
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

void b3EnableFileCaching(int enable);

std::string LoadFromCachedOrFromObj(
    std::vector<tinyobj::shape_t>& shapes,   // [output]
    const char* filename,
    const char* mtl_basepath);

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath);

// added for compound shapes
btCompoundShape* LoadMeshFromObjs(const char* relativeFileName, const
char* materialPrefixPath);

#endif //LOAD_MESH_FROM_OBJ_H

