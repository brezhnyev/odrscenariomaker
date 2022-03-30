#include "Waypath.h"
#include "thirdparty/cpp-spline/CatmullRom.h"

#include <GL/gl.h>

#include <sstream>
#include <iostream>

using namespace Eigen;
using namespace std;

void Waypath::drawGeometry()
{
    float psz;
    glGetFloatv(GL_POINT_SIZE, &psz);
    glPointSize(2);
    glPushMatrix();
    glTranslatef(0,0,0.1f);
    glBegin(GL_POINTS);
    for (auto && c : m_smoothPath)
        glVertex3f(c.getPosition().x(), c.getPosition().y(), c.getPosition().z());
    glEnd();
    glPopMatrix();
    glPointSize(psz);
}

void Waypath::draw()
{
    drawGeometry();
    for (auto && c : m_children) c.second->draw();
}


void Waypath::updateSmoothPath()
{
    if (m_children.size() < 2)
        return;

    m_smoothPath.clear();

	CatmullRom curve;
    const int STEPS = 100;
	curve.set_steps(STEPS); // generate STEPS interpolate points between the last 4 way points

    // specifics of the cpp-spline library: we need to add the first and last waypoints twice:
    Waypoint * firstWP = static_cast<Waypoint*>(m_children.begin()->second);
    curve.add_way_point(Vector(firstWP->getPosition().x(), firstWP->getPosition().y(), firstWP->getPosition().z()));
    for (auto && c : m_children)
    {
        Waypoint * wp = static_cast<Waypoint*>(c.second);
        curve.add_way_point(Vector(wp->getPosition().x(), wp->getPosition().y(), wp->getPosition().z()));
    }
    Waypoint * lastWP = static_cast<Waypoint*>(m_children.rbegin()->second);
    curve.add_way_point(Vector(lastWP->getPosition().x(), lastWP->getPosition().y(), lastWP->getPosition().z()));

    auto it1 = m_children.begin();
    auto it2 = m_children.begin(); advance(it2, 1);
    for (int i = 0, s = 0; i < curve.node_count(); ++i, ++s)
    {
        if (STEPS == s) // we skip the last point in the trajectory section since its the same as the first point in the next section
        {
            s = 0;
            ++it1; ++it2;
            ++i;
            continue;
        }
        Waypoint * wp1 = static_cast<Waypoint*>(it1->second);
        Waypoint * wp2 = static_cast<Waypoint*>(it2->second);
        float speed = (1.0f - (float)s/STEPS)*wp1->getSpeed() + (float)s/STEPS*wp2->getSpeed();
        WaypointSmoothed wp(Vector3f(curve.node(i).x, curve.node(i).y, curve.node(i).z), speed);
		m_smoothPath.push_back(wp);
	}
}

Vector3f Waypath::getInitialDirection()
{
    if (m_smoothPath.size() < 2)
        return Vector3f(0,0,0);
    Vector3f dir = m_smoothPath[1].getPosition() - m_smoothPath[0].getPosition();
    dir.normalize();
    return dir;
}

Vector3f Waypath::getInitialPosition()
{
    if (m_smoothPath.size() < 1)
        return Vector3f(0,0,0);
    return m_smoothPath[0].getPosition();
}

bool Waypath::getNext(Vector3f & pos, Vector3f & dir, float & targetSpeed, float currentSpeed, int fps)
{
    // find the nearest next point:
    float dist = 100000;
    int index = 0;

    // check if we are in the nearabouts of the final waypoint:
    if ((pos - m_smoothPath.back().getPosition()).norm() <= currentSpeed/fps)
        return false;

    // find the closest point to the pos:
    for (int i = 0; i < m_smoothPath.size(); ++i)
    {
        auto d = (pos - m_smoothPath[i].getPosition()).norm();
        if (d < dist)
        {
            index = i;
            dist = d;
        }
    }

    if (index == m_smoothPath.size() - 2)
        return false;

    targetSpeed = m_smoothPath[index].getSpeed();

    dir = (m_smoothPath[index+1].getPosition() - m_smoothPath[index].getPosition()).normalized();

    return true;
}

string Waypath::serialize() const
{
    Eigen::Vector3f v; float val; int counter = 0;
    stringstream ss;
    for (auto && child : m_children)
    {
        auto p = dynamic_cast<Waypoint*>(child.second)->getPosition();
        ss << p.x() << " " << p.y() << " " << p.z() << " ";
    }
    return ss.str();
}