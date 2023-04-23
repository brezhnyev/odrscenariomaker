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

#include "../scenario.h"
#include "../Serializer.h"

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

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <signal.h>

#include <eigen3/Eigen/Eigen>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool isStopped;
int server_fd, new_socket, valread;
#define PORT 12345 

void prepareServer()
{
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[1024] = {0}; 
       
    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    }
}


int main(int argc, char ** argv)
{
    prepareServer();

    Serializer ser;
    Scenario scenario = ser.deserialize_yaml(argv[1]);
    isStopped = false;

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Town02");

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    auto scenario_vehicles = scenario.children();
    const int number_of_vehicles = scenario_vehicles.size();
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
    vector<ShrdPtrActor> vehicles; vehicles.reserve(number_of_vehicles);

    cc::Vehicle::Control control;

    int i = 0;
    for (auto it = scenario_vehicles.begin(); it != scenario_vehicles.end(); ++it, ++i)
    {
        Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(it->second);
        auto blueprint = (*world.GetBlueprintLibrary()->Filter(scenario_vehicle.getName()))[0];
        if (blueprint.ContainsAttribute("color"))
        {
            auto &attribute = blueprint.GetAttribute("color");
            blueprint.SetAttribute("color", scenario_vehicle.colorToString());
        }
        // Spawn the vehicle.
        auto actor = world.TrySpawnActor(blueprint, spawn_points[i%number_of_vehicles]);
        if (!actor)
        {
            cout << "Failed to spawn actor ------" << endl;
            continue;
        }
        cout << "Spawned " << actor->GetDisplayId() << '\n';
        // Set the scenario start position:
        Waypath & waypath = *dynamic_cast<Waypath*>(scenario_vehicle.children().begin()->second);
        auto wit = waypath.children().begin();
        auto wp1 = dynamic_cast<Waypoint*>(wit->second); wp1->flipY(); // Since Carla is LEFT handed - flip Y
        ++wit;
        auto wp2 = dynamic_cast<Waypoint*>(wit->second); wp2->flipY(); // Since Carla is LEFT handed - flip Y
        auto dir = wp2->getPosition() - wp1->getPosition();
        auto yaw = (atan2(dir.y(), dir.x()))*90/M_PI_2;
        cg::Transform transform(cg::Location(wp1->getPosition().x(), wp1->getPosition().y(), wp1->getPosition().z()), cg::Rotation(0,yaw,0));
        actor->SetTransform(transform);
        auto vehicle = static_cast<cc::Vehicle*>(actor.get());
        control.throttle = 1.0f;
        vehicle->ApplyControl(control);
        vehicle->SetSimulatePhysics();
        vehicles.push_back(actor);
    }
    world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    // Set spectator trf by the first waypoint of the first vehicle:
    Vehicle & first_vehicle = *dynamic_cast<Vehicle*>(scenario_vehicles.begin()->second);
    auto spectator = world.GetSpectator();
    //transform.location -= 10.0f * transform.GetForwardVector();
    cg::Transform transform = vehicles[0]->GetTransform();
    transform.location.z += 30.0f;
    //transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -80.0f;
    spectator->SetTransform(transform);


    float SPEED = 10.0f;

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
            auto * actor = &vehicles[0];
            for (auto it = scenario_vehicles.begin(); it != scenario_vehicles.end(); ++it, ++actor)
            {
                auto vehicle = static_cast<cc::Vehicle*>((*actor).get());

                auto trf = vehicle->GetTransform();
                auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
                auto speed =  vehicle->GetVelocity().Length();

                Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(it->second);
                Waypath & waypath = *dynamic_cast<Waypath*>(scenario_vehicle.children().begin()->second);

                // Get the next waypoints in some distance away:
                Eigen::Vector3f peigen(trf.location.x, -trf.location.y, trf.location.z);
                if (!waypath.getNext(peigen))
                {
                    isStopped = true;
                    break;
                }
                cg::Transform p(cg::Location(peigen.x(), -peigen.y(), peigen.z()));
                auto dir = (p.location - trf.location).MakeUnitVector();
                auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
                auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;

                control.steer = sign * arc.Length();
                vehicle->ApplyControl(control);
                // The stronger is the curvature the lower speed:
                float R = abs(3 * tan(M_PI_2 - control.steer)); // 3 is the ~length between axes
                float acc = speed*speed/R; // get centrifusual acceleration
                brake(0.01f*acc, vehicle);

                stringstream ss;
                ss 
                    << scenario_vehicle.getID()
                    << " " << trf.location.x 
                    << " " << trf.location.y 
                    << " " << trf.location.z 
                    << " " << trf.rotation.yaw;
                send(new_socket, ss.str().c_str(), ss.str().size(), 0 ); 
            }
            
            world.Tick(carla::time_duration(1s));

        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    for (auto v : vehicles) v->Destroy();
    send(new_socket, "*", 1, 0 );
    usleep(1e6);
    shutdown(new_socket, SHUT_RDWR);
}

