#pragma once

#include "Waypath.h"
#include "Vehicle.h"

#include <eigen3/Eigen/Eigen>

#include <vector>

class Scenario : public Selectable
{
public:
    std::string getType() const override { return "Scenario"; }

    int addWaypoint(Eigen::Vector3f);

    Actor *     getActiveActor();
    Waypath *   getActiveWaypath();
    Waypoint *  getActiveWaypoint();

    int         getActiveActorID();
    int         getActiveWaypathID();
    int         getActiveWaypointID();

    void        setRosbagFile(const std::string & text)     { m_rosbagFile = text;  }
    void        setRosbagTopics(std::vector<std::string> topics)    { m_rosbagTopics = topics; }
    void        setRosbagOffset(float offset)               { m_rosbagOffset=offset;}
    void        setTownName(std::string name)               { m_townName = name;    }
    std::string getRosbagFile()                             { return m_rosbagFile;  }
    std::vector<std::string> getRosbagTopics()              { return m_rosbagTopics;}
    float       getRosbagOffset()                           { return m_rosbagOffset;}
    std::string getTownName()                               { return m_townName;    }

protected:
    std::string  m_rosbagFile;
    std::vector<std::string>  m_rosbagTopics;
    std::string m_townName;
    float        m_rosbagOffset{0.0f};
};  