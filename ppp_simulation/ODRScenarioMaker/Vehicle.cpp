#include "Vehicle.h"

#include <fstream>
#include <iostream>

#include <GL/gl.h>

using namespace std;
using namespace Eigen;

void Vehicle::draw()
{
    glDisable(GL_TEXTURE_2D);

    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z() + 1);

    glPushMatrix();
    glRotatef(m_yaw,0,0,1);

    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_QUADS);
    glVertex3f(-2, -1, 0);
    glVertex3f( 2, -1, 0);
    glVertex3f( 2,  1, 0);
    glVertex3f(-2,  1, 0);
    glEnd();

    glPopMatrix();
    glPopMatrix();
}