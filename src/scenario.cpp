#include "scenario.h"
#include "Walker.h"
#include "Vehicle.h"
#include "Waypoint.h"
#include "Waypath.h"
#include "Camera.h"

#include <vector>

using namespace std;
using namespace Eigen;

// get active elements -------------
Actor * Scenario::getActiveActor()
{
    return dynamic_cast<Actor*>(getActiveChild(1));
}

Waypath * Scenario::getActiveWaypath()
{
    return dynamic_cast<Waypath*>(getActiveChild(2));
}

Waypoint * Scenario::getActiveWaypoint()
{
    return dynamic_cast<Waypoint*>(getActiveChild(3));
}

void Scenario::to_yaml(YAML::Node & parent)
{
    parent["type"] = "Scenario";
    parent["townname"] = m_townName;
    parent["rosbagfile"] = m_rosbagFile;
    for (auto && rostopic : m_rosbagTopics)
    {
        YAML::Node topic;
        topic["topic"] = rostopic;
        parent["rosbagtopics"].push_back(topic);
    }
    parent["rosbagoffset"] = m_rosbagOffset;
    YAML::Node actors;
    parent["components"] = actors;
    for (auto && child : m_children) child->to_yaml(actors);
}

void Scenario::from_yaml(const YAML::Node & node)
{
    makeUndoSnapshot(this);
    s_allowUndo = false;
    loadScenario(node);
    s_allowUndo = true;
}

void Scenario::loadScenario(const YAML::Node & node)
{
    m_townName = node["townname"].as<string>();
    m_rosbagFile = node["rosbagfile"].as<string>();
    auto topics = node["rosbagtopics"];
    vector<string> rostopics;
    for (auto && topic : topics)
    {
        m_rosbagTopics.push_back(topic["topic"].as<string>());
    }
    m_rosbagOffset = node["rosbagoffset"].as<float>();
    auto components = node["components"];
    for (auto it = components.begin(); it != components.end(); ++it)
    {
        auto child = *it;
        if (child["type"].as<string>() == "Vehicle")
        {
            Vehicle * vehicle = new Vehicle(this);
            vehicle->from_yaml(child);
        }
        else if (child["type"].as<string>() == "Walker")
        {
            Walker * walker = new Walker(this);
            walker->from_yaml(child);
        }
        else if (child["type"].as<string>() == "Camera")
        {
            Camera * camera = new Camera(this);
            camera->from_yaml(child);
        }
    }
}

void Scenario::undo()
{
    if (s_undo.empty())
        return;

    YAML::Node root;
    to_yaml(root);
    stringstream ss; ss << root;
    s_redo.push(ss.str());

    s_allowUndo = false;
    clear();
    root = YAML::Load(s_undo.top());
    loadScenario(root);
    s_allowUndo = true;
    s_undo.pop();
}

void Scenario::redo()
{
    if (s_redo.empty())
        return;

    YAML::Node root;
    to_yaml(root);
    stringstream ss; ss << root;
    s_undo.push(ss.str());

    s_allowUndo = false;
    clear();
    root = YAML::Load(s_redo.top());
    loadScenario(root);
    s_allowUndo = true;
    s_redo.pop();
}