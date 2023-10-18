#pragma once

#include "Actor.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

class Vehicle : public Actor
{
public:
    Vehicle(Selectable * parent, std::string name = "vehicle.audi.a2") : Actor(name, parent)
    {
    }
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const;
    std::string getType() const override { return "Vehicle"; }
    ADDVAR(protected, bool, isEgo, false);
};