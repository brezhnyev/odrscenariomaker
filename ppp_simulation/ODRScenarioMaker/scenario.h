#pragma once

#include "Canvas.h"
#include "Waypath.h"
#include "Vehicle.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario : public Selectable
{
public:
    Scenario();
    void init();
    void draw() override;
    void drawWithNames() override;
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

private:
    Canvas m_canvas;
};  