#include "Camera.h"

#include <GL/gl.h>

void Camera::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + 0.5);

    glPushMatrix();
    glRotatef(m_ori[2],0,0,1);

    Eigen::Vector3i color(0, 255, 0);

    if (m_selected)
    {
        glColor3f(float(255-color[0])/255, float(255-color[1])/255, float(255-color[2])/255);
        glPushMatrix();
        glScalef(1.5f, 1.5f, 1.0f);
        glBegin(GL_QUADS);
        // we mame a tapering reducing Y
        glVertex3f(-m_bbox[0], -m_bbox[1]*0.2, -0.05);
        glVertex3f( m_bbox[0], -m_bbox[1], -0.05);
        glVertex3f( m_bbox[0],  m_bbox[1], -0.05);
        glVertex3f(-m_bbox[0],  m_bbox[1]*0.2, -0.05);
        glEnd();
        glPopMatrix();
    }

    glColor3f(float(color[0])/255, float(color[1])/255, float(color[2])/255);
    glBegin(GL_QUADS);
    // we mame a tapering reducing Y
    glVertex3f(-m_bbox[0], -m_bbox[1]*0.2, 0);
    glVertex3f( m_bbox[0], -m_bbox[1], 0);
    glVertex3f( m_bbox[0],  m_bbox[1], 0);
    glVertex3f(-m_bbox[0],  m_bbox[1]*0.2, 0);
    glEnd();

    glPopMatrix();
    glPopMatrix();
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