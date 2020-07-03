#pragma once

#include "Waypath.h"
#include "Vehicle.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario : public Selectable
{
public:
    std::string getType() const override { return "Scenario"; }

    int addVehicle();
    int addWaypath();
    int addWaypoint(Eigen::Vector3f);

    int delActor(int id);
    int delWaypath(int id);
    int delWaypoint(int id);

    Actor *      getActiveActor();
    Waypath *    getActiveWaypath();
    Waypoint *   getActiveWaypoint();

    int          getActiveActorID();
    int          getActiveWaypathID();
    int          getActiveWaypointID();
};  