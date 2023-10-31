#pragma once

#include "globals.h"
#include "Selectable.h"

#include <eigen3/Eigen/Eigen>
#include <vector>

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

class Actor;
class Waypath;
class Waypoint;

class Scenario : public Selectable
{
public:
    Scenario(Selectable * parent) : Selectable(parent) {};
    std::string getType() const override { return "Scenario"; }
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;

    Actor *     getActiveActor();
    Waypath *   getActiveWaypath();
    Waypoint *  getActiveWaypoint();

    void undo();
    void redo();

    ADDVAR(protected, std::string, rosbagFile, "");
    ADDVAR(protected, std::vector<std::string>, rosbagTopics, std::vector<std::string>());
    ADDVAR(protected, std::string, townName, "");
    ADDVAR(protected, float, rosbagOffset, 0.0f);
    ADDVAR(protected, std::string, scenarioFileName, "");

private:
    void loadScenario(const YAML::Node & node);
};  