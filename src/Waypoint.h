#pragma once

#include "globals.h"
#include "Selectable.h"

#include <atomic>

#include <eigen3/Eigen/Eigen>

/** WaypointNonSelectable class.
 * The class corresponds to the points found after application the Catmul-Rom algorithm on a set of Waypoints.
 * The WaypointNonSelectable waypoints are not going to be selected (mouse pick up) and listed in the tree view
 * so they also dont have the ID and dont have children.
 */
class WaypointNonSelectable
{
public:
    WaypointNonSelectable() {}
    WaypointNonSelectable(Eigen::Vector3f pos, float speed) : m_pos(pos), m_speed(speed) {}
    void flipY() { m_pos[1] = -m_pos[1]; }

    ADDVAR(protected, Eigen::Vector3f, pos, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    ADDVAR(protected, float, speed, 0.0f);
};


/** Waypoint class.
 * The class corresponds to the sparse Waypoints in Carla that specify the desired path of an object.
 */
class Waypoint : public Selectable, public WaypointNonSelectable
{
public:
    Waypoint(Selectable * parent) : Selectable(parent) {};
    Waypoint(Eigen::Vector3f pos, float speed, Selectable * parent);
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const override;
    std::string getType() const override { return "Waypoint"; }
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;
};
