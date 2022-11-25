#pragma once

#include "Actor.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

class Vehicle : public Actor
{
public:
    Vehicle(std::string name = "vehicle.audi.a2") : Actor(name), m_color(Eigen::Vector3i(50, 100, 150))
    {
    }
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const;
    std::string getType() const override { return "Vehicle"; }
    std::string colorToString();
    Eigen::Vector3i stringToColor();

    Eigen::Vector3i m_color;
};