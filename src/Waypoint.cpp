#include "Waypoint.h"

#include <GL/gl.h>

using namespace std;

Waypoint::Waypoint(Eigen::Vector3f pos, float speed, Selectable * parent) : Selectable(parent), m_pos(pos), m_speed(speed)
{
    // generically we push back the new object (see Selectable c-tor)
    // For Waypoint we will augment this logics by following:
    // If Waypath is selected - the new Waypoint will be placed in front of the waypoints
    // If existing Waypoint is selected - the new Waypoint will be placed after it
    if (parent->children().size() == 1)
        return;

    Selectable * object = parent->getSelected();
    if (object->getType() == "Waypath")
    {
        parent->children().push_front(parent->children().back());
    }
    else
    {
        auto it = find_if(parent->children().begin(), parent->children().end(), [object](Selectable * o){ return o == object; });
        parent->children().insert(it+1, parent->children().back());
        m_speed = dynamic_cast<Waypoint*>(object)->get_speed();
    }
    parent->children().pop_back();
}

void Waypoint::drawGeometry() const
{
    glDisable(GL_TEXTURE_2D);

    // get current color (is set when the vehicle is drawn)
    float cc[4];
    glGetFloatv(GL_CURRENT_COLOR, cc);

    glColor3f(1.0f-cc[0], 1.0f-cc[1], 1.0f-cc[2]);

    float w = 1.0f;
    glPushMatrix();
    glTranslatef(m_pos.x(), m_pos.y(), m_pos.z()+0.5);
    if (m_selected)
    {
        glBegin(GL_QUADS);
        glVertex3f(- w, - w, -0.05);
        glVertex3f(+ w, - w, -0.05);
        glVertex3f(+ w, + w, -0.05);
        glVertex3f(- w, + w, -0.05);
        glEnd();
    }

    w = 0.5f;
    glColor3f(cc[0], cc[1], cc[2]);
    glBegin(GL_QUADS);
    glVertex3f(- w, - w, 0);
    glVertex3f(+ w, - w, 0);
    glVertex3f(+ w, + w, 0);
    glVertex3f(- w, + w, 0);
    glEnd();
    glPopMatrix();
}

void Waypoint::draw() const
{
    drawGeometry();
}

void Waypoint::drawWithNames() const
{
    glPushName(m_id);
    drawGeometry();
    glPopName();
}

void Waypoint::to_yaml(YAML::Node & parent)
{
    YAML::Node node;
    node["type"] = getType();
    YAML::Node location;
    location["x"] = m_pos.x();
    location["y"] = m_pos.y();
    location["z"] = m_pos.z();
    node["location"] = location;
    node["speed"] = m_speed;
    parent.push_back(node);
}


void Waypoint::from_yaml(const YAML::Node & node)
{
    auto location = node["location"];
    float x = location["x"].as<float>();
    float y = location["y"].as<float>();
    float z = location["z"].as<float>();
    m_pos = Eigen::Vector3f(x,y,z);
    m_speed = node["speed"].as<float>();
}