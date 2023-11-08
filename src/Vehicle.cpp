#include "Vehicle.h"
#include "Camera.h"

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
    glRotatef(m_ori[2],0,0,1);
    glScalef(m_bbox[0], m_bbox[1], m_bbox[2]);

    if (m_selected)
        glLineWidth(5);
    else
        glLineWidth(1);
    glColor3f(float(255-m_color[0])/255, float(255-m_color[1])/255, float(255-m_color[2])/255);

    for (auto && c : box)
    {
        glBegin(GL_LINE_LOOP);
        glVertex3f(c[0], c[1], c[2]);
        glVertex3f(c[3], c[4], c[5]);
        glVertex3f(c[6], c[7], c[8]);
        glVertex3f(c[9], c[10],c[11]);
        glEnd();
    }

    glColor3f(float(m_color[0])/255, float(m_color[1])/255, float(m_color[2])/255);
    for (auto && c : box)
    {
        glBegin(GL_QUADS);
        glVertex3f(c[0], c[1], c[2]);
        glVertex3f(c[3], c[4], c[5]);
        glVertex3f(c[6], c[7], c[8]);
        glVertex3f(c[9], c[10], c[11]);
        glEnd();
    }

    // Front:
    glLineWidth(5);
    glBegin(GL_LINES);
    glVertex3f(1.0f, 0.0f, 0.0f);
    glVertex3f(2.0f, 0.0f, 0.0f);
    glEnd();
    glLineWidth(1);

    glPopMatrix();
}

void Vehicle::draw() const
{
    drawGeometry();
    Selectable::draw();
}

void Vehicle::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
    for (auto && child : m_children) child->drawWithNames();
}

void Vehicle::to_yaml(YAML::Node & parent)
{
    Actor::to_yaml(parent);
    // specific Vehicle fields:
    parent[parent.size()-1]["isEgo"] = m_isEgo;
}

void Vehicle::from_yaml(const YAML::Node & node)
{
    Actor::from_yaml(node);
    // specific Vehicle fields:
    m_isEgo = node["isEgo"].as<bool>();
}