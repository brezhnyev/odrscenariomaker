#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/detail/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/WalkerAIController.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/rpc/WheelPhysicsControl.h>
#include <carla/rpc/VehicleLightState.h>
#include <carla/rpc/WalkerBoneControl.h>

#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <map>
#include <deque>

#include <iostream>
#include <signal.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool isStopped;

void sighandler(int sig)
{
    switch(sig)
    {
        case SIGINT:
        cout << "\n***Handling Ctrl+C signal...***" << endl;
        cout.flush();
        isStopped = true;
        break;
    }
    cout << sig << endl;
}

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator)
{
    assert(range.size() > 0u);
    uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(forward<RNG>(generator))]; // KB: this can fail for map
}

int main(int argc, const char *argv[])
{
    // This test is adopted for Town02 so any other one will place the vehicle in improper position
    string town_name = "Town02";
    if (argc > 1)
    {
        string town_name = argv[1];
    }
    isStopped = false;
    signal(SIGINT, sighandler);

    mt19937_64 rng((random_device())());

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto m_world = client.LoadWorld(town_name);
    auto waypoint_topology = m_world.GetMap()->GetTopology();
    
    // Synchronous mode:
    auto defaultSettings = m_world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    m_world.ApplySettings(wsettings);
    m_world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    const int number_of_vehicles = 1;
    auto blueprints = m_world.GetBlueprintLibrary()->Filter("vehicle.volkswagen.t2");
    vector<ShrdPtrActor> vehicles; vehicles.reserve(number_of_vehicles);

    for (int i = 0; i < number_of_vehicles; ++i)
    {
        auto blueprint = RandomChoice(*blueprints, rng);
        // Find a valid spawn point.

        // Randomize the blueprint.
        if (blueprint.ContainsAttribute("color"))
        {
            auto &attribute = blueprint.GetAttribute("color");
            blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), rng));
        }
        if (i==0)
            blueprint.SetAttribute("role_name", "hero");
        else
            blueprint.SetAttribute("role_name", "autopilot");

        // Spawn the vehicle.
        auto actor = m_world.TrySpawnActor(blueprint, cg::Transform(cg::Location(193.699, 250, 0.3), cg::Rotation(0,-90,0)));
        if (!actor) continue;
        // Finish and store the vehicle
        vehicles.push_back(actor);
        cout << "Spawned " << vehicles.back()->GetDisplayId() << '\n';
    }

    auto vehicle = static_cast<cc::Vehicle*>(vehicles[0].get());
    vehicle->SetTransform(cg::Transform(cg::Location(193.699, 250, 0), cg::Rotation(0,-90,0)));
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);

    // Spawn walkers:
    const int number_of_walkers = 1;
    vector<ShrdPtrActor> walkers; walkers.reserve(number_of_walkers);

    auto w_bp = m_world.GetBlueprintLibrary()->Filter("walker.pedestrian.0004"); // "Filter" returns BluePrintLibrary (i.e. wrapper about container of ActorBlueprints)
    auto wc_bp = m_world.GetBlueprintLibrary()->Find("controller.ai.walker"); // "Find" returns pointer to the ActorBlueprint

    for (int i = 0; i < number_of_walkers;)
    {
        auto walker_bp = RandomChoice(*w_bp, rng);
        if (walker_bp.ContainsAttribute("is_invincible")) walker_bp.SetAttribute("is_invincible", "false");
        auto walker = m_world.TrySpawnActor(walker_bp, cg::Transform(cg::Location(180, 245, 1), cg::Rotation(0,-90,0)));
        if (!walker) continue;
        ++i;

        // Store the walker and its controller
        walkers.push_back(walker);
        cout << "Spawned " << walkers.back()->GetDisplayId() << '\n';
    }
    m_world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    auto walker = static_cast<cc::Walker*>(walkers[0].get());
    auto wcontrol = walker->GetWalkerControl();
    m_world.SetPedestriansCrossFactor(0.0f);

    cg::Transform transform(cg::Location(180, 245, 2), cg::Rotation(0,-90,0));
    auto spectator = m_world.GetSpectator();
    transform.location += 15.0f * transform.GetForwardVector();
    transform.location.z += 5.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -45.0f;
    spectator->SetTransform(transform);

    float SPEED = 10.0f;
    map<int, std::vector<carla::SharedPtr<cc::Waypoint>>> paths;
    int pathID = -1;
    float DIST = 5.0f; // how far away we scan the waypoints ahead of the vehicle
    bool turnLeft = true;

    vehicle->SetSimulatePhysics();

    // Comments to the below block:
    // The GetWaypoint(position) returns the closest waypoint to the requested position
    // This function returns 1 waypoint of the SAME road IF the vehicle is NOT in junction
    // HOWEVER IF the vehicle IS in the junction the above function can return a waypoint of ANY crossing road:
    // auto waypoint = m_world.GetMap()->GetWaypoint(vehicle->GetLocation());

    // So there are several major values here:
    // 1. vehicle->GetTransform() - the current position of the vehicle
    // 2. waypoint (see above) - the closest waypoint to the vehicle
    // 3. waypoints are the next waypoints from the waypoint DIST away ahead from it. Should be array of size 1 if the scan is NOT in junction.
    // 4. paths is a container of possible paths (i.e. waypoints) in the junction

    auto brake = [&](float strength, cc::Vehicle* vehicle)
    {
        auto ls = vehicle->GetLightState();
        auto speed =  vehicle->GetVelocity().Length();
        const float threshold = 0.05f;

        if (strength < threshold && speed < SPEED)
        {
            control.brake = 0.0f;
            control.throttle = exp(-speed/SPEED);
            ls = crpc::VehicleLightState::LightState::None;
        }
        else
        {
            strength = max(strength, threshold);
            control.brake = strength;
            control.throttle = 0.0f;
            ls = crpc::VehicleLightState::LightState::Brake;
        }

        vehicle->SetLightState(ls);
        vehicle->ApplyControl(control);
    };

    while (!isStopped)
    {
        try
        {
            wcontrol.speed = 1.5f;
            wcontrol.direction = cg::Vector3D(0,-1,0);
            walker->ApplyControl(wcontrol);

            auto waypoint = m_world.GetMap()->GetWaypoint(vehicle->GetLocation());
            auto trf = vehicle->GetTransform();
            auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
            auto speed =  vehicle->GetVelocity().Length();

            // vehicle enters the junction, figure out which of the alternative ways is the turn to the left
            if (waypoint->IsJunction() && pathID == -1)
            {
                map<float, int> indices; // from turning left to turning right
                for (auto && path : paths)
                {
                    auto p = path.second.back();
                    auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
                    auto z = dir.x * heading.y - dir.y * heading.x;
                    indices[z] = path.first;
                }
                if (turnLeft) pathID = indices.rbegin()->second;
                else pathID = indices.begin()->second;
            }

            // vehicle leaves the junction
            if (!waypoint->IsJunction() && pathID != -1)
            {
                pathID = -1;
                paths.clear();
                turnLeft = !turnLeft; // alternate turn left and turn right
                brake(0.0f, vehicle);
                vehicle->ApplyControl(control);
            }

            // If the vehicle in the junction: the m_world.GetMap()->GetWaypoint(vehicle->GetLocation())
            // is not any more reliable, since it can return ANY closest waypoint and so put the vehicle on a wrong track
            // So use now the reviously collected paths[pathID] i.e. left turn
            if  (waypoint->IsJunction())
            {
                // find the closest to the the points in the junction:
                float dist = 10000;
                int index = 0;
                auto l1 = waypoint->GetTransform().location;
                for (int i = 0; i < paths[pathID].size(); ++i)
                {
                    auto l2 = paths[pathID][i]->GetTransform().location;
                    auto s = (l2 - l1).Length();
                    if (s < dist)
                    {
                        index = i;
                        dist = s;
                    }
                }
                waypoint = paths[pathID][index]; // this is the closest waypoint to the vehicle on the PROPER path
            }

            // Get the next waypoints in some distance away:
            auto waypoints = waypoint->GetNext(DIST);
            map<float, int> indices; // from turning to eft to turning to right
            for (int i = 0; i < waypoints.size(); ++i)
            {
                auto p = waypoints[i];
                auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
                auto z = dir.x * heading.y - dir.y * heading.x;
                indices[z] = i;
                if (p->IsJunction())
                {
                    paths[p->GetRoadId()].push_back(p);
                    vehicle->ApplyControl(control);
                }
            }

            // // we choose the left most point to compute the direction of movement and the angular speed:
            auto p = waypoints[indices.rbegin()->second]; // turn left
            if (!turnLeft) p = waypoints[indices.begin()->second]; // turn right
            auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
            auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
            auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;

            // COMMENT:
            // here the p MAY happen to be a waypoint from a IMPROPER track!!!
            // This is especially possible when the waypoints are very close to the beginning of the junction
            // where the tracks are almost overlapping
            // This is NOT critical for heading the vehicle at this moment since the vehicle is now ~DIST meters away from the junction

            control.steer = sign * arc.Length();
            vehicle->ApplyControl(control);
            // The stronger is the curvature the lower speed:
            float R = abs(3 * tan(M_PI_2 - control.steer)); // 3 is the ~length between axes
            float acc = speed*speed/R; // get centrifusual acceleration
            brake(0.01f*acc, vehicle);
            if (p->IsJunction()) control.throttle = 0.0f;
            
            m_world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    m_world.ApplySettings(defaultSettings);
    for (auto v : vehicles) v->Destroy();
    for (auto w : walkers) w->Destroy();

    return 0;
}



