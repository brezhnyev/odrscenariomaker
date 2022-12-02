#include "Waypoint.h"

#include <GL/gl.h>

using namespace std;

Waypoint::Waypoint(Eigen::Vector3f pos, float speed, Selectable * parent) : Selectable(parent), m_pos(pos), m_speed(speed) {}

void Waypoint::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    // get current color (is set when the vehicle is drawn)
    float cc[4];
    glGetFloatv(GL_CURRENT_COLOR, cc);

    glColor3f(1.0f-cc[0], 1.0f-cc[1], 1.0f-cc[2]);

    float w = 1.0f;
    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z()+0.5);
    if (m_selected)
    {
        glBegin(GL_QUADS);
        glVertex3f(- w, - w, -0.05);
        glVertex3f(+ w, - w, -0.05);
        glVertex3f(+ w, + w, -0.05);
        glVertex3f(- w, + w, -0.05);
        glEnd();
    }

    w = 0.5f;
    glColor3f(cc[0], cc[1], cc[2]);
    glBegin(GL_QUADS);
    glVertex3f(- w, - w, 0);
    glVertex3f(+ w, - w, 0);
    glVertex3f(+ w, + w, 0);
    glVertex3f(- w, + w, 0);
    glEnd();
    glPopMatrix();
}

void Waypoint::draw() const
{
    drawGeometry();
}

void Waypoint::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
}