#include "ptrafficmanager.h"

#include <carla/geom/Transform.h>
#include <carla/client/Map.h> // KB: the header includes the boost::next somewhere needed for yaml, so just leave it here

#include <yaml-cpp/yaml.h>

#include <string>

using namespace std;
using namespace carla::geom;
using namespace carla::client;

PTrafficManager::PTrafficManager(World & world, string confname)
{
    YAML::Node config = YAML::LoadFile(confname.c_str());

    auto vehicles = config["vehicles"];
    for (auto && v : vehicles)
    {
        string name = v["name"].as<string>();
        Location l(v["location"]["x"].as<float>(), v["location"]["y"].as<float>(), v["location"]["z"].as<float>());
        Rotation r(v["rotation"]["pitch"].as<float>(), v["rotation"]["yaw"].as<float>(), v["rotation"]["roll"].as<float>());
        float speed = v["speed"].as<float>();
        vector<string> behaviour;
        for (auto && b : v["behaviour"]) behaviour.push_back(b.as<string>());
        m_actors.push_back(new PVehicle(world, name, Transform(l,r), speed, behaviour));
    }

    auto walkers = config["walkers"];
    for (auto && w : walkers)
    {
        string name = w["name"].as<string>();
        Location l(w["location"]["x"].as<float>(), w["location"]["y"].as<float>(), w["location"]["z"].as<float>());
        Rotation r(w["rotation"]["pitch"].as<float>(), w["rotation"]["yaw"].as<float>(), w["rotation"]["roll"].as<float>());
        float speed = w["speed"].as<float>();
        vector<string> behaviour;
        for (auto && b : w["behaviour"]) behaviour.push_back(b.as<string>());
        m_actors.push_back(new PWalker(world, name, Transform(l,r), speed, behaviour));
    }
}
