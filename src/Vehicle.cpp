#include "Vehicle.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include <GL/gl.h>

using namespace std;
using namespace Eigen;

static const float box[][12] = {
    -1,-1,-1, +1,-1,-1, +1,+1,-1, -1,+1,-1,
    -1,-1,+1, +1,-1,+1, +1,+1,+1, -1,+1,+1,
    -1,-1,-1, +1,-1,-1, +1,-1,+1, -1,-1,+1,
    -1,+1,-1, +1,+1,-1, +1,+1,+1, -1,+1,+1,
    -1,-1,-1, -1,-1,+1, -1,+1,+1, -1,+1,-1,
    +1,-1,-1, +1,-1,+1, +1,+1,+1, +1,+1,-1
};

void Vehicle::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + m_bbox[2]);

    glPushMatrix();
    glRotatef(m_ori[2],0,0,1);

    if (m_selected)
    {
        glColor3f(float(255-m_color[0])/255, float(255-m_color[1])/255, float(255-m_color[2])/255);
        
        glLineWidth(5);
        for (auto && c : box)
        {
            glBegin(GL_LINE_LOOP);
            glVertex3f(c[0]*m_bbox[0], c[1]*m_bbox[1], c[2]*m_bbox[2]);
            glVertex3f(c[3]*m_bbox[0], c[4]*m_bbox[1], c[5]*m_bbox[2]);
            glVertex3f(c[6]*m_bbox[0], c[7]*m_bbox[1], c[8]*m_bbox[2]);
            glVertex3f(c[9]*m_bbox[0], c[10]*m_bbox[1],c[11]*m_bbox[2]);
            glEnd();
        }
        glLineWidth(1);
    }

    glColor3f(float(m_color[0])/255, float(m_color[1])/255, float(m_color[2])/255);
    for (auto && c : box)
    {
        glBegin(GL_QUADS);
        glVertex3f(c[0]*m_bbox[0], c[1]*m_bbox[1], c[2]*m_bbox[2]);
        glVertex3f(c[3]*m_bbox[0], c[4]*m_bbox[1], c[5]*m_bbox[2]);
        glVertex3f(c[6]*m_bbox[0], c[7]*m_bbox[1], c[8]*m_bbox[2]);
        glVertex3f(c[9]*m_bbox[0], c[10]*m_bbox[1], c[11]*m_bbox[2]);
        glEnd();
    }

    glPopMatrix();
    glPopMatrix();
}

void Vehicle::draw() const
{
    drawGeometry();
    for (auto && child : m_children) child.second->draw();
}

void Vehicle::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
    for (auto && child : m_children) child.second->drawWithNames();
}