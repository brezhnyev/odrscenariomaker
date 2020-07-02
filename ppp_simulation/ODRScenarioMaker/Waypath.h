#pragma once

#include "Waypoint.h"

#include <vector>
#include <deque>

class Waypath : public Selectable
{
public:
    int delChild(int id) override;
    bool getNext(Eigen::Vector3f & pos);
    std::string serialize();
};