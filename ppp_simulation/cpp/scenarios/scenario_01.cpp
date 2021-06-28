#include "ptrafficmanager.h"

#include <carla/client/Client.h>
#include <carla/client/TimeoutException.h>
#include <carla/rpc/EpisodeSettings.h>

#include <stdexcept>
#include <string>
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

int main(int argc, const char *argv[])
{
    // This test is adopted for Town02 so any other one will place the vehicle in improper position
    string confname;
    string town_name = "Town02";
    if (argc > 1) confname = argv[1];
    if (argc > 2) town_name = argv[2];

    isStopped = false;
    signal(SIGINT, sighandler);

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld(town_name);
    
    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    PTrafficManager tm(world, confname);
    // tm.AddActor(new PVehicle(world, cg::Transform(cg::Location(193.699, 250, 0.3), cg::Rotation(0,-90,0))));
    // tm.AddActor(new PWalker(world, cg::Transform(cg::Location(180, 245, 1), cg::Rotation(0,-90,0))));

    cg::Transform transform(cg::Location(180, 245, 2), cg::Rotation(0,-90,0));
    auto spectator = world.GetSpectator();
    transform.location += 15.0f * transform.GetForwardVector();
    transform.location.z += 5.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -45.0f;
    spectator->SetTransform(transform);

    while (!isStopped)
    {
        try
        {
            tm.Tick();
            world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));

    return 0;
}



