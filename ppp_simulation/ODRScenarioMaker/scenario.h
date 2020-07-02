#pragma once

#include "Waypath.h"
#include "Vehicle.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario
{
public:
    Scenario();
    void draw();
    void drawWithNames();

    int addVehicle();
    int addWaypath();
    int addWaypoint(Eigen::Vector3f);

    void delActor(int id);
    void delWaypath(int id);
    void delWaypoint();
    void select(int id);

    Selectable * getSelectable(int id);
    Waypath *    getActiveWaypath();
    int          getActiveWaypathID();
    int          getActiveActorID();

public:

private:
    int m_activeActor;
    std::map<int, Actor*>    m_actors;
};  