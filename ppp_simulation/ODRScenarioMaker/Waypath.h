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
    bool getNext(Eigen::Vector3f & pos, float & targetSpeed);
};