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

#include "../Waypath.h"
#include <eigen3/Eigen/Eigen>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool isStopped;


int main(int argc, char ** argv)
{
    Waypath waypath;
    {
        Eigen::Vector3f v; float val; int counter = 0;
        stringstream ss(argv[1]);
        while(ss >> val)
        {
            if (counter%3 == 0) v.x() = val;
            if (counter%3 == 1) v.y() =-val;
            if (counter%3 == 2) 
            {
                v.z() = val;
                waypath.pushWaypoint(v);
            }
            ++counter;
        }
    }

    isStopped = false;

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Town02");

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings);
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    auto blueprints = world.GetBlueprintLibrary()->Filter("vehicle.volkswagen.t2");
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
    auto blueprint = (*blueprints)[0];
    // Spawn the vehicle.
    auto wp1 = waypath.getWaypoints()[0];
    auto wp2 = waypath.getWaypoints()[2];
    auto dir = wp2.getPosition() - wp1.getPosition();
    auto yaw = (atan2(dir.y(), dir.x()))*90/M_PI_2;
    cg::Transform transform(cg::Location(wp1.getPosition().x(), wp1.getPosition().y(), wp1.getPosition().z()), cg::Rotation(0,yaw,0));
    ShrdPtrActor actor = world.TrySpawnActor(blueprint, spawn_points[0]);
    if (!actor) return 1;
    // Finish and store the vehicle
    cout << "Spawned " << actor->GetDisplayId() << '\n';

    //cg::Transform transform(cg::Location(173.699, 237, 0), cg::Rotation(0,-180,0));
    actor->SetTransform(transform);
    auto spectator = world.GetSpectator();
    //transform.location -= 10.0f * transform.GetForwardVector();
    transform.location.z += 30.0f;
    //transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -80.0f;
    spectator->SetTransform(transform);

    auto vehicle = static_cast<cc::Vehicle*>(actor.get());
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);

    world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

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
    // auto waypoint = world.GetMap()->GetWaypoint(vehicle->GetLocation());

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
            auto trf = vehicle->GetTransform();
            auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
            auto speed =  vehicle->GetVelocity().Length();

            // Get the next waypoints in some distance away:
            Eigen::Vector3f peigen(trf.location.x, trf.location.y, trf.location.z);
            if (!waypath.getNext(peigen)) break;
            cg::Transform p(cg::Location(peigen.x(), peigen.y(), peigen.z()));
            auto dir = (p.location - trf.location).MakeUnitVector();
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
            
            world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings);
    actor->Destroy();

}


