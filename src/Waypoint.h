#pragma once

#include "globals.h"
#include "Selectable.h"

#include <atomic>

#include <eigen3/Eigen/Eigen>


/** Waypoint class.
 * The class corresponds to the sparse Waypoints in Carla that specify the desired path of an object.
 */
class Waypoint : public Selectable
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

    ADDVAR(protected, float, speed, 0.0f);
    ADDVAR(protected, Eigen::Vector3f, pos, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
};
