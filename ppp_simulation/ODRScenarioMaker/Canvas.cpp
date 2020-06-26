#include "Canvas.h"
#include <GL/gl.h>

using namespace std;
using namespace Eigen;

Canvas::Canvas() {}

Canvas::Canvas(string texname, QRect rect) : m_rect(rect), m_image(QString(texname.c_str()))
{
    m_vertices.push_back(Vector3f(rect.x(), rect.y(), 0.0f));
    m_vertices.push_back(Vector3f(rect.x(), rect.y() + rect.height(), 0.0f));
    m_vertices.push_back(Vector3f(rect.x() + rect.width(), rect.y() + rect.height(), 0.0f));
    m_vertices.push_back(Vector3f(rect.x() + rect.width(), rect.y(), 0.0f));

    m_texcoord.push_back(Vector2f(0.0f, 0.0f));
    m_texcoord.push_back(Vector2f(0.0f, 1.0f));
    m_texcoord.push_back(Vector2f(1.0f, 1.0f));
    m_texcoord.push_back(Vector2f(1.0f, 0.0f));
}

void Canvas::init()
{
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glEnable(GL_TEXTURE_2D);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_image.width(), m_image.height(), 0, GL_BGRA, GL_UNSIGNED_BYTE, m_image.bits());
}

void Canvas::draw()
{
    glPushMatrix();
    glEnable(GL_TEXTURE_2D);
    glColor3f(1.0f, 1.0f, 1.0f);
    glScalef(1,-1,1);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, &m_vertices.front());
    glTexCoordPointer(2, GL_FLOAT, 0, &m_texcoord.front());
    glDrawArrays(GL_QUADS, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glPopMatrix();
}

void Canvas::drawWithNames()
{
    glPushName(0);
    draw();
    glPopName();
}