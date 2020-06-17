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
    
    // ofstream ofs1("./" + town_name + "/waypoints1.ply");
    // for (auto && wp : waypoint_topology)
    // {
    //     auto l = wp.first->GetTransform().location;
    //     ofs1 << l.x << " " << l.y << " " << l.z << endl;
    // }
    // ofs1.close();
    // ofstream ofs2("./" + town_name + "/waypoints2.ply");
    // for (auto & wp : waypoint_topology)
    // {
    //     auto l = wp.second ->GetTransform().location;
    //     ofs2 << l.x << " " << l.y << " " << l.z << endl;
    // }
    // ofs2.close();
    // auto waypoints = m_world.GetMap()->GetRecommendedSpawnPoints();
    // ofstream ofs("./" + town_name + "/spawn_points.ply");
    // for (auto && sp : waypoints)
    // {
    //     auto l = sp.location;
    //     ofs << l.x << " " << l.y << " " << l.z << endl;
    // }
    // ofs.close();

    // Synchronous mode:
    auto defaultSettings = m_world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    m_world.ApplySettings(wsettings);
    m_world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    const int number_of_vehicles = 1;
    auto blueprints = m_world.GetBlueprintLibrary()->Filter("vehicle.volkswagen.t2");
    auto spawn_points = m_world.GetMap()->GetRecommendedSpawnPoints();
    vector<ShrdPtrActor> vehicles; vehicles.reserve(number_of_vehicles);

    for (int i = 0; i < number_of_vehicles; ++i)
    {
        auto blueprint = RandomChoice(*blueprints, rng);
        // Find a valid spawn point.
        auto transform = RandomChoice(spawn_points, rng);

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
        auto actor = m_world.TrySpawnActor(blueprint, transform);
        if (!actor) continue;
        // Finish and store the vehicle
        vehicles.push_back(actor);
        cout << "Spawned " << vehicles.back()->GetDisplayId() << '\n';
    }

    cg::Transform transform(cg::Location(193.699, 270, 0), cg::Rotation(0,-90,0));
    //cg::Transform transform(cg::Location(173.699, 237, 0), cg::Rotation(0,-180,0));
    vehicles[0]->SetTransform(transform);
    auto spectator = m_world.GetSpectator();
    //transform.location -= 10.0f * transform.GetForwardVector();
    transform.location.z += 50.0f;
    //transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -60.0f;
    spectator->SetTransform(transform);

    auto vehicle = static_cast<cc::Vehicle*>(vehicles[0].get());
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);

    // auto traffic_manager = client.GetInstanceTM(8000); //KB: the port
    // traffic_manager.SetGlobalDistanceToLeadingVehicle(2.0);
    // traffic_manager.SetSynchronousMode(true);
    // traffic_manager.RegisterVehicles(vehicles);

    // adding physics control
    // auto physics = vehicle->GetPhysicsControl();
    // vector<crpc::WheelPhysicsControl>wheels(4);
    // //physics.wheels = wheels;
    // physics.moi = 1.0;
    // physics.damping_rate_full_throttle = 0.0;
    // physics.use_gear_autobox = true;
    // physics.gear_switch_time = 0.5;
    // physics.clutch_strength = 10;
    // physics.mass = 5000;
    // physics.drag_coefficient = 0.25;
    // vehicle->ApplyPhysicsControl(physics);


    // PYTHON example:
    // front_left_wheel  = carla.WheelPhysicsControl(tire_friction=4.5, damping_rate=1.0, max_steer_angle=70.0, radius=30.0)
    // front_right_wheel = carla.WheelPhysicsControl(tire_friction=2.5, damping_rate=1.5, max_steer_angle=70.0, radius=25.0)
    // rear_left_wheel   = carla.WheelPhysicsControl(tire_friction=1.0, damping_rate=0.2, max_steer_angle=0.0,  radius=15.0)
    // rear_right_wheel  = carla.WheelPhysicsControl(tire_friction=1.5, damping_rate=1.3, max_steer_angle=0.0,  radius=20.0)

    // wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]

    // # Change Vehicle Physics Control parameters of the vehicle
    // physics_control = vehicle.get_physics_control()

    // physics_control.torque_curve = [carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)]
    // physics_control.max_rpm = 10000
    // physics_control.moi = 1.0
    // physics_control.damping_rate_full_throttle = 0.0
    // physics_control.use_gear_autobox = True
    // physics_control.gear_switch_time = 0.5
    // physics_control.clutch_strength = 10
    // physics_control.mass = 10000
    // physics_control.drag_coefficient = 0.25
    // physics_control.steering_curve = [carla.Vector2D(x=0, y=1), carla.Vector2D(x=100, y=1), carla.Vector2D(x=300, y=1)]
    // physics_control.wheels = wheels

    m_world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    float SPEED = 10.0f;
    map<int, std::vector<carla::SharedPtr<cc::Waypoint>>> paths;
    int pathID = -1;
    const float DIST = 6.0f; // how far away we scan the waypoints ahead of the vehicle
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

    while (!isStopped)
    {
        try
        {
            auto waypoint = m_world.GetMap()->GetWaypoint(vehicle->GetLocation());
            auto trf = vehicle->GetTransform();
            auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
            auto speed =  vehicle->GetVelocity().Length();
            if (speed > SPEED)
            {
                control.throttle = 0.5f;
                vehicle->ApplyControl(control);
            }
            else
            {
                control.brake = 0.0f;
                vehicle->ApplyControl(control);
            }

            // vehicle enters the junction, figure out which of the alternative ways is the turn to the left
            if (waypoint->IsJunction() && pathID == -1)
            {
                 map<float, int> indices; // from turning to eft to turning to right
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
                control.brake = 0.0f;
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
                }
            }

            // // we choose the left most point to compute the direction of movement and the angular speed:
            auto p = waypoints[indices.rbegin()->second]; // turn left
            if (!turnLeft) p = waypoints[indices.begin()->second]; // turn right
            auto dir = (p->GetTransform().location - trf.location).MakeUnitVector();
            // vehicle->SetVelocity(dir*SPEED);
            auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
            auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;
            // vehicle->SetAngularVelocity(cg::Vector3D(0.0f,0.0f, SPEED * sign * arc.Length()/0.0333));

            // COMMENT:
            // here the p MAY happen to be a waypoint from a IMPROPER track!!!
            // This is especially possible when the waypoints are very close to the beginning of the junction
            // where the tracks are almost overlapping
            // This is NOT critical for heading the vehicle at this moment since the vehicle is now ~DIST meters away from the junction

            control.steer = sign * arc.Length();
            vehicle->ApplyControl(control);

            // The stronger is the curvature the lower speed:
            // get centrifusual acceleration:
            auto ac = speed*arc.Length();
            control.brake = ac/30;
            cout << control.brake << endl;
            // However not lower than 1/10 th of the speed:
            if (vehicle->GetVelocity().Length() < 0.5*SPEED) control.brake = 0.0f;
            vehicle->ApplyControl(control);
            
            m_world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    m_world.ApplySettings(defaultSettings);
    for (auto v : vehicles) v->Destroy();

    return 0;
}



