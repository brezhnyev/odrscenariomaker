#include "World3D.h"
#include "thirdparty/ObjLoader.h"
#include <GL/gl.h>

using namespace std;

static objl::Loader s_objLoader;

World3D::World3D()
{
    s_objLoader.LoadFile("/home/kbrezhnyev/DATA/aorta_paris/Connex/connex.obj");
}

World3D::World3D(const string & objpath)
{

}

void World3D::init()
{
    m_objPointsList = glGenLists(1);
    glNewList(m_objPointsList, GL_COMPILE);
    glBegin(GL_POINTS);
    for (auto && mesh : s_objLoader.LoadedMeshes)
    {
        for (auto && v : mesh.Vertices)
        {
            glVertex3f(v.Position.X, -v.Position.Z, v.Position.Y);
            //cout << v.Position.X << " " << v.Position.Y << " " << v.Position.Z << endl;
        }
    }
    glEnd();
    glEndList();
}

void World3D::draw()
{
    glColor3f(1.0f, 0.0f, 0.0f);
    glCallList(m_objPointsList);
}