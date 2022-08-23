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
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + 1);

    glPushMatrix();
    glRotatef(m_ori[2],0,0,1);

    glColor3f(float(m_color[0])/255, float(m_color[1])/255, float(m_color[2])/255);
    glBegin(GL_QUADS);
    glVertex3f(-2, -1, 0);
    glVertex3f( 2, -1, 0);
    glVertex3f( 2,  1, 0);
    glVertex3f(-2,  1, 0);
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