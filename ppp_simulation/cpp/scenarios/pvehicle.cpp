#include "pactor.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>

#include <iostream>

using namespace carla::geom;
using namespace carla::client;
using namespace std;

PVehicle::PVehicle(carla::client::World & world, std::string name, carla::geom::Transform trf, float speed, std::vector<string> behaviour) 
: PActor(world, name, trf, speed, behaviour)
{
    auto blueprints = world.GetBlueprintLibrary()->Filter(name);
    if (!blueprints->empty())
    {
        auto blueprint = (*blueprints)[0];

        // if (blueprint.ContainsAttribute("color"))
        // {
        //     auto &attribute = blueprint.GetAttribute("color");
        //     blueprint.SetAttribute("color", carla::sensor::data::Color(255,0,0));
        // }
        // if (i==0)
        //     blueprint.SetAttribute("role_name", "hero");
        // else
        //     blueprint.SetAttribute("role_name", "autopilot");

        // Spawn the vehicle.
        m_actor = world.TrySpawnActor(blueprint, trf);
        cout << "Spawned " << m_actor->GetDisplayId() << '\n';

        auto vehicle = static_cast<Vehicle*>(m_actor.get());
        //m_control = vehicle->GetControl(); // KB: Control should be used created by default c-tor, otherwise the vehicle does not move
        m_control.throttle = 1.0f;
        vehicle->ApplyControl(m_control);
        vehicle->SetSimulatePhysics();
        
        m_pathID = -1;
        m_dist = 5.0f; // how far away we scan the waypoints ahead of the vehicle
        m_turnLeft = true;
    }
}

void PVehicle::Tick()
{
    auto vehicle = static_cast<Vehicle*>(m_actor.get());
    auto brake = [&](float strength, carla::client::Vehicle* vehicle)
    {
        auto ls = vehicle->GetLightState();
        auto speed =  vehicle->GetVelocity().Length();
        const float threshold = 0.05f;

        if (strength < threshold && speed < m_speed)
        {
            m_control.brake = 0.0f;
            m_control.throttle = exp(-speed/m_speed);
            ls = carla::rpc::VehicleLightState::LightState::None;
        }
        else
        {
            strength = max(strength, threshold);
            m_control.brake = strength;
            m_control.throttle = 0.0f;
            ls = carla::rpc::VehicleLightState::LightState::Brake;
        }

        vehicle->SetLightState(ls);
        vehicle->ApplyControl(m_control);
    };

    auto waypoint = m_world.GetMap()->GetWaypoint(vehicle->GetLocation());
    auto trf = vehicle->GetTransform();
    auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
    auto speed = vehicle->GetVelocity().Length();

    // vehicle enters the junction, figure out which of the alternative ways is the turn to the left
    if (waypoint->IsJunction() && m_pathID == -1)
    {
        map<float, int> indices; // from turning left to turning right
        for (auto &&path : m_paths)
        {
            auto p = path.second.back();
            auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
            auto z = dir.x * heading.y - dir.y * heading.x;
            indices[z] = path.first;
        }
        if (m_turnLeft)
            m_pathID = indices.rbegin()->second;
        else
            m_pathID = indices.begin()->second;
    }

    // vehicle leaves the junction
    if (!waypoint->IsJunction() && m_pathID != -1)
    {
        m_pathID = -1;
        m_paths.clear();
        m_turnLeft = !m_turnLeft; // alternate turn left and turn right
        brake(0.0f, vehicle);
        vehicle->ApplyControl(m_control);
    }

    // If the vehicle in the junction: the m_world.GetMap()->GetWaypoint(vehicle->GetLocation())
    // is not any more reliable, since it can return ANY closest waypoint and so put the vehicle on a wrong track
    // So use now the reviously collected paths[m_pathID] i.e. left turn
    if (waypoint->IsJunction())
    {
        // find the closest to the the points in the junction:
        float dist = 10000;
        int index = 0;
        auto l1 = waypoint->GetTransform().location;
        for (int i = 0; i < m_paths[m_pathID].size(); ++i)
        {
            auto l2 = m_paths[m_pathID][i]->GetTransform().location;
            auto s = (l2 - l1).Length();
            if (s < dist)
            {
                index = i;
                dist = s;
            }
        }
        waypoint = m_paths[m_pathID][index]; // this is the closest waypoint to the vehicle on the PROPER path
    }

    // Get the next waypoints in some distance away:
    auto waypoints = waypoint->GetNext(m_dist);
    map<float, int> indices; // from turning to eft to turning to right
    for (int i = 0; i < waypoints.size(); ++i)
    {
        auto p = waypoints[i];
        auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
        auto z = dir.x * heading.y - dir.y * heading.x;
        indices[z] = i;
        if (p->IsJunction())
        {
            m_paths[p->GetRoadId()].push_back(p);
            vehicle->ApplyControl(m_control);
        }
    }

    // // we choose the left most point to compute the direction of movement and the angular speed:
    auto p = waypoints[indices.rbegin()->second]; // turn left
    if (!m_turnLeft)
        p = waypoints[indices.begin()->second]; // turn right
    auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
    auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
    auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;

    // COMMENT:
    // here the p MAY happen to be a waypoint from a IMPROPER track!!!
    // This is especially possible when the waypoints are very close to the beginning of the junction
    // where the tracks are almost overlapping
    // This is NOT critical for heading the vehicle at this moment since the vehicle is now ~DIST meters away from the junction

    m_control.steer = sign * arc.Length();
    vehicle->ApplyControl(m_control);
    // The stronger is the curvature the lower speed:
    float R = abs(3 * tan(M_PI_2 - m_control.steer)); // 3 is the ~length between axes
    float acc = speed * speed / R;                  // get centrifusual acceleration
    brake(0.01f * acc, vehicle);
}