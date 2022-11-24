#include "Vehicle.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include <GL/gl.h>

using namespace std;
using namespace Eigen;


void Vehicle::drawGeometry()
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + 0.5);

    glPushMatrix();
    glRotatef(m_ori[2],0,0,1);

    if (m_selected)
    {
        glColor3f(float(255-m_color[0])/255, float(255-m_color[1])/255, float(255-m_color[2])/255);
        glPushMatrix();
        glScalef(1.5f, 1.5f, 1.0f);
        glBegin(GL_QUADS);
        glVertex3f(-m_bbox[0], -m_bbox[1], -0.05);
        glVertex3f( m_bbox[0], -m_bbox[1], -0.05);
        glVertex3f( m_bbox[0],  m_bbox[1], -0.05);
        glVertex3f(-m_bbox[0],  m_bbox[1], -0.05);
        glEnd();
        glPopMatrix();
    }

    glColor3f(float(m_color[0])/255, float(m_color[1])/255, float(m_color[2])/255);
    glBegin(GL_QUADS);
    glVertex3f(-m_bbox[0], -m_bbox[1], 0);
    glVertex3f( m_bbox[0], -m_bbox[1], 0);
    glVertex3f( m_bbox[0],  m_bbox[1], 0);
    glVertex3f(-m_bbox[0],  m_bbox[1], 0);
    glEnd();

    glPopMatrix();
    glPopMatrix();
}

void Vehicle::draw()
{
    drawGeometry();
    for (auto && child : m_children) child.second->draw();
}

std::string Vehicle::colorToString()
{
    return to_string(m_color[0]) + "," + to_string(m_color[1]) + "," + to_string(m_color[2]);
}

Eigen::Vector3i Vehicle::stringToColor()
{
    return Eigen::Vector3i(0,0,0);
}

void Vehicle::drawWithNames()
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
    for (auto && child : m_children) child.second->drawWithNames();
}