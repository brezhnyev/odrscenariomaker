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

    for (auto && wp : m_waypaths) wp.second.draw();
}

void Vehicle::drawWithNames()
{
    for (auto && wp : m_waypaths) wp.second.drawWithNames();
}

int Vehicle::addWaypath()
{
    Waypath wp;
    m_waypaths[wp.getID()] = wp;
    m_activeWaypath = wp.getID();
    return m_activeWaypath;
}

int Vehicle::addWaypoint(Eigen::Vector3f p)
{
    int id = m_waypaths[m_activeWaypath].addWaypoint(p);
    return id;
}

void Vehicle::delWaypath(int id)
{
    if (m_waypaths.empty()) return;

    m_waypaths.erase(id);
    m_activeWaypath = 0;
}

void Vehicle::delWaypoint()
{
    m_waypaths[m_activeWaypath].delWaypoint();
}

bool Vehicle::select(int id)
{
    // otherwise selet the waypoint only:
    m_activeWaypath = 0;
    m_selected = false;

    for (auto && path : m_waypaths)
    {
        if (path.second.select(id))
        {
            m_activeWaypath = path.second.getID();
        }
    }

    // if path is selected then select all points on the path:
    if (id == m_id)
    {
        for (auto && path : m_waypaths) path.second.select(path.second.getID());
        m_selected = true;
    }

    return m_selected || m_activeWaypath;
}

Selectable * Vehicle::getChild(int id)
{
    Selectable * selection = nullptr;

    for (auto && path : m_waypaths)
    {
        if (path.second.getID() == id)
        {
            return &path.second;
        }
        selection = path.second.getChild(id);
        if (selection) return selection;
    }
    return selection;
}

Waypath * Vehicle::getActiveWaypath()
{
    return &m_waypaths[m_activeWaypath];
}