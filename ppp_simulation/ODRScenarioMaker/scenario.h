#pragma once

#include "Waypath.h"
#include "Vehicle.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario : public Selectable
{
public:
    int addVehicle();
    int addWaypath();
    int addWaypoint(Eigen::Vector3f);

    void delActor(int id);
    void delWaypath(int id);
    void delWaypoint(int id);

    Actor *      getActiveActor();
    Waypath *    getActiveWaypath();
    Waypoint *   getActiveWaypoint();

    int          getActiveActorID();
    int          getActiveWaypathID();
    int          getActiveWaypointID();
};  