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

    Actor *     getActiveActor();
    Waypath *   getActiveWaypath();
    Waypoint *  getActiveWaypoint();

    int         getActiveActorID();
    int         getActiveWaypathID();
    int         getActiveWaypointID();

    void        setRosbagFile(const std::string & text)     { m_rosbagFile = text;  }
    void        setRosbagTopic(const std::string & text)    { m_rosbagTopic = text; }
    void        setRosbagOffset(float offset)               { m_rosbagOffset=offset;}
    std::string getRosbagFile()                             { return m_rosbagFile;  }
    std::string getRosbagTopic()                            { return m_rosbagTopic; }
    float       getRosbagOffset()                           { return m_rosbagOffset;}

protected:
    std::string  m_rosbagFile;
    std::string  m_rosbagTopic;
    float        m_rosbagOffset{0.0f};
};  