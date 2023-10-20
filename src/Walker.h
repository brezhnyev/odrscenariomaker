#pragma once

#include "Actor.h"
#include "Waypath.h"

#include <eigen3/Eigen/Eigen>

class Walker : public Actor
{
public:
    Walker(Selectable * parent, std::string name = "walker.pedestrian.0001") : Actor(name, parent)
    {
        set_bbox(Eigen::Vector3f(.25f, .25f, 1.0f));
    }
    void draw() const override;
    void drawWithNames() const override;
    void drawGeometry() const;
    std::string getType() const override { return "Walker"; }
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;
};