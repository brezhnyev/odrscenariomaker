#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public Selectable
{
public:
    Waypath(Selectable * parent) : Selectable(parent) {}
    std::string serialize() const;
    std::string getType() const override { return "Waypath"; }
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;

    bool getNext(Eigen::Vector3f & pos, Eigen::Vector3f & dir, float & targetSpeed, float currentSpeed, int fps);
    Eigen::Vector3f getStartingDirection();
    Eigen::Vector3f getEndingDirection();
    Eigen::Vector3f getStartingPosition();
    Eigen::Vector3f getEndingPosition();
    void updateSmoothPath();
    void draw() const override;
    void drawGeometry() const override;

private:
    std::vector<WaypointNonSelectable> m_smoothPath;
};