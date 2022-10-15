#pragma once

#include "Selectable.h"

#include <atomic>

#include <eigen3/Eigen/Eigen>

/** Waypoint class.
 * The class corresponds to the sparse Waypoints in Carla that specify the desired path of an object.
 */
class Waypoint : public Selectable
{
public:
    Waypoint(Eigen::Vector3f pos, float speed);
    void draw() override;
    void drawWithNames() override;
    void drawGeometry() override;
    std::string getType() const override { return "Waypoint"; }

    Eigen::Vector3f getPosition() { return m_pos; }
    void  setPosition(Eigen::Vector3f val) { m_pos = val; }
    void flipY() { m_pos[1] = -m_pos[1]; }
    float getSpeed() { return m_speed; }
    void setSpeed(float speed) { m_speed = speed; }

private:
    Eigen::Vector3f     m_pos;
    float               m_speed;
};


/** Smoothed Waypoint class.
 * The class corresponds to the points found after application the Catmul-Rom algorithm on a set of Waypoints.
 * The Smoothed waypoints are not going to be selected (mouse pick up) and listed in the tree view
 * so they also dont have the ID and dont have children.
 */
class WaypointSmoothed
{
public:
    WaypointSmoothed(Eigen::Vector3f pos, float speed) : m_pos(pos), m_speed(speed) {}
    Eigen::Vector3f getPosition() { return m_pos; }
    void  setPosition(Eigen::Vector3f val) { m_pos = val; }
    void flipY() { m_pos[1] = -m_pos[1]; }
    float getSpeed() { return m_speed; }
    void setSpeed(float speed) { m_speed = speed; }

private:
    Eigen::Vector3f     m_pos;
    float               m_speed;
};