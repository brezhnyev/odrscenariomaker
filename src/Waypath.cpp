#include "Waypath.h"
#include "thirdparty/cpp-spline/CatmullRom.h"

#include <GL/gl.h>

#include <sstream>
#include <iostream>

using namespace Eigen;
using namespace std;

void Waypath::drawGeometry() const
{
    float psz;
    glGetFloatv(GL_POINT_SIZE, &psz);
    if (m_selected)
        glPointSize(4);
    else
        glPointSize(2);
    glPushMatrix();
    glTranslatef(0,0,0.5f);
    glBegin(GL_POINTS);
    for (auto && c : m_smoothPath)
    {
        auto pos = c.get_pos();
        glVertex3f(pos.x(), pos.y(), pos.z());
    }
    glEnd();
    glPopMatrix();
    glPointSize(psz);
}

void Waypath::draw() const
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
    curve.add_way_point(Vector(firstWP->get_pos().x(), firstWP->get_pos().y(), firstWP->get_pos().z()));
    for (auto && c : m_children)
    {
        Waypoint * wp = static_cast<Waypoint*>(c.second);
        curve.add_way_point(Vector(wp->get_pos().x(), wp->get_pos().y(), wp->get_pos().z()));
    }
    Waypoint * lastWP = static_cast<Waypoint*>(m_children.rbegin()->second);
    curve.add_way_point(Vector(lastWP->get_pos().x(), lastWP->get_pos().y(), lastWP->get_pos().z()));

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
        float speed = (1.0f - (float)s/STEPS)*wp1->get_speed() + (float)s/STEPS*wp2->get_speed();
        WaypointNonSelectable wp(Vector3f(curve.node(i).x, curve.node(i).y, curve.node(i).z), speed);
		m_smoothPath.push_back(wp);
	}
}

Vector3f Waypath::getStartingDirection()
{
    if (m_smoothPath.size() < 2)
        return Vector3f(0,0,0);
    Vector3f dir = m_smoothPath[1].get_pos() - m_smoothPath[0].get_pos();
    dir.normalize();
    return dir;
}

Vector3f Waypath::getEndingDirection()
{
    if (m_smoothPath.size() < 2)
        return Vector3f(0,0,0);
    Vector3f dir = (m_smoothPath.end()-1)->get_pos() - (m_smoothPath.end()-2)->get_pos();
    dir.normalize();
    return dir;
}

Vector3f Waypath::getStartingPosition()
{
    if (m_smoothPath.size() < 1)
        return Vector3f(0,0,0);
    return m_smoothPath[0].get_pos();
}

Vector3f Waypath::getEndingPosition()
{
    if (m_smoothPath.size() < 1)
        return Vector3f(0,0,0);
    return m_smoothPath.back().get_pos();
}

bool Waypath::getNext(Vector3f & pos, Vector3f & dir, float & targetSpeed, float currentSpeed, int fps)
{
    if (m_smoothPath.empty())
    {
        return false;
    }
    // find the nearest next point:
    float dist = 100000;
    int index = 0;

    // check if we are in the nearabouts of the final waypoint:
    if ((pos - m_smoothPath.back().get_pos()).norm() <= currentSpeed/fps)
        return false;

    // find the closest point to the pos:
    for (int i = 0; i < m_smoothPath.size(); ++i)
    {
        auto d = (pos - m_smoothPath[i].get_pos()).norm();
        if (d < dist)
        {
            index = i;
            dist = d;
        }
    }

    if (index >= m_smoothPath.size() - 2)
        return false;

    targetSpeed = m_smoothPath[index].get_speed();

    // We also need to force the vehicle move into the trajectory in case the dir is parallel to it
    dir = (m_smoothPath[index+1].get_pos() - m_smoothPath[index].get_pos()).normalized() + 0.5f*(m_smoothPath[index].get_pos() - pos);
    dir.normalize();

    return true;
}

string Waypath::serialize() const
{
    Eigen::Vector3f v; float val; int counter = 0;
    stringstream ss;
    for (auto && child : m_children)
    {
        auto p = dynamic_cast<Waypoint*>(child.second)->get_pos();
        ss << p.x() << " " << p.y() << " " << p.z() << " ";
    }
    return ss.str();
}