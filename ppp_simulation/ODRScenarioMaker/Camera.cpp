#include "Camera.h"

#include <GL/gl.h>

static const float box[][12] = {
    0,0,0, +1,-1,-1, +1,+1,-1, 0,0,0,
    0,0,0, +1,-1,+1, +1,+1,+1, 0,0,0,
    0,0,0, +1,-1,-1, +1,-1,+1, 0,0,0,
    0,0,0, +1,+1,-1, +1,+1,+1, 0,0,0,
    0,0,0, 0,0,0, 0,0,0, 0,0,0,
    +1,-1,-1, +1,-1,+1, +1,+1,+1, +1,+1,-1
};

void Camera::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix(); // parent
    Actor * parent = dynamic_cast<Actor*>(m_parent);
    if (parent)
    {
        Eigen::Vector3f pos = parent->getPos();
        glTranslatef(pos[0], pos[1], pos[2]);
        Eigen::Vector3f ori = parent->getOri();
        glRotatef(ori[2],0,0,1);
        glRotatef(0,ori[1],0,1);
        glRotatef(0,0,ori[0],1);
    }

    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + 0.5);
    glRotatef(m_ori[2],0,0,1);
    glRotatef(0,m_ori[1],0,1);
    glRotatef(0,0,m_ori[0],1);

    Eigen::Vector3i color(0, 255, 0);

    if (m_selected)
    {
        glColor3f(float(255-color[0])/255, float(255-color[1])/255, float(255-color[2])/255);
        glPushMatrix();
        glScalef(1.5f, 1.5f, 1.5f);
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
    }

    glColor3f(float(color[0])/255, float(color[1])/255, float(color[2])/255);
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
    glPopMatrix(); // parent transform (if any)
}

void Camera::draw() const
{
    drawGeometry();
    for (auto && child : m_children) child.second->draw();
}

void Camera::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
    for (auto && child : m_children) child.second->drawWithNames();
}