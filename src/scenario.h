#pragma once

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
    Scenario(const Scenario &); // copy c-tor (need to redefine since we can load)
    Scenario & operator=(const Scenario &);
    std::string getType() const override { return "Scenario"; }
    void to_yaml(YAML::Node & parent) override;
    void from_yaml(const YAML::Node & node) override;

    Actor *     getActiveActor();
    Waypath *   getActiveWaypath();
    Waypoint *  getActiveWaypoint();

    void        setRosbagFile(const std::string & text)     { m_rosbagFile = text;  }
    void        setRosbagTopics(std::vector<std::string> topics)    { m_rosbagTopics = topics; }
    void        setRosbagOffset(float offset)               { m_rosbagOffset=offset;}
    void        setTownName(std::string name)               { m_townName = name;    }
    std::string getRosbagFile()                             { return m_rosbagFile;  }
    std::vector<std::string> getRosbagTopics()              { return m_rosbagTopics;}
    float       getRosbagOffset()                           { return m_rosbagOffset;}
    std::string getTownName()                               { return m_townName;    }

protected:
    std::string                 m_rosbagFile;
    std::vector<std::string>    m_rosbagTopics;
    std::string                 m_townName;
    float                       m_rosbagOffset{0.0f};
};  