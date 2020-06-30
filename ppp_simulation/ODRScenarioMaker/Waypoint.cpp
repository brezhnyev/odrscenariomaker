#include "Waypoint.h"

#include <GL/gl.h>

using namespace std;

Waypoint::Waypoint(Eigen::Vector3f pos, float speed) : Selectable(), m_pos(pos), m_speed(speed) {}

void Waypoint::draw()
{
    glDisable(GL_TEXTURE_2D);

    float w = 0.25f;
    glColor3f(1.0f, 0.0f, 0.0f);

    if (m_selected)
    {
        glBegin(GL_QUADS);
        glVertex3f(m_pos.x() - w, m_pos.y() - w, m_pos.z()+0.05);
        glVertex3f(m_pos.x() + w, m_pos.y() - w, m_pos.z()+0.05);
        glVertex3f(m_pos.x() + w, m_pos.y() + w, m_pos.z()+0.05);
        glVertex3f(m_pos.x() - w, m_pos.y() + w, m_pos.z()+0.05);
        glEnd();
    }

    w = 0.20f;
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_QUADS);
    glVertex3f(m_pos.x() - w, m_pos.y() - w, m_pos.z()+0.1);
    glVertex3f(m_pos.x() + w, m_pos.y() - w, m_pos.z()+0.1);
    glVertex3f(m_pos.x() + w, m_pos.y() + w, m_pos.z()+0.1);
    glVertex3f(m_pos.x() - w, m_pos.y() + w, m_pos.z()+0.1);
    glEnd();
}

void Waypoint::drawWithNames()
{
    glPushName(m_id);
    draw();
    glPopName();
}

bool Waypoint::select(int id)
{ 
    m_selected = id == m_id ? true : false; 
    return m_selected;
}