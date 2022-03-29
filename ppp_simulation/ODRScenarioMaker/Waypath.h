#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public Selectable
{
public:
    int delChild(int id) override; // must be overriden for Waypath
    std::string serialize() const;
    std::string getType() const override { return "Waypath"; }
    bool getNext(Eigen::Vector3f & pos, Eigen::Vector3f & dir, float & targetSpeed, float currentSpeed, int fps);
    Eigen::Vector3f getInitialDirection();
    Eigen::Vector3f getInitialPosition();
    void updateSmoothPath();
    void draw() override;
    void drawWithNames() override {};
    void drawGeometry() override;

private:
    std::vector<WaypointSmoothed> m_smoothPath;
};