#include "World3D.h"
#include "thirdparty/ObjLoader.h"
#include <GL/gl.h>
#include <iostream>

using namespace std;

static objl::Loader s_objLoader;

World3D::World3D(const string & objfile) : Selectable(nullptr)
{
    if (!objfile.empty())
        s_objLoader.LoadFile(objfile);
}
                                                                                                                                                                                                                                                                                                                                                
World3D::~World3D()
{
    glDeleteLists(m_objPointsList, 1);
}

void World3D::init()
{
    for (auto && mesh : s_objLoader.LoadedMeshes)
        cout << mesh.MeshMaterial.map_Kd << endl;
}

void World3D::draw() const
{
    for (auto && mesh : s_objLoader.LoadedMeshes)
    {
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(objl::Vertex), &mesh.Vertices[0].Position);
        glTexCoordPointer(2, GL_FLOAT, sizeof(objl::Vertex), &mesh.Vertices[0].TextureCoordinate );
        glDrawElements(GL_TRIANGLES, mesh.Indices.size(), GL_UNSIGNED_INT, &mesh.Indices[0]);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
}