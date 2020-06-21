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

#include "ptrafficmanager.h"

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

    PTrafficManager tm;
    tm.AddActor(new PVehicle(m_world, cg::Transform(cg::Location(193.699, 250, 0.3), cg::Rotation(0,-90,0))));
    tm.AddActor(new PWalker(m_world, cg::Transform(cg::Location(180, 245, 1), cg::Rotation(0,-90,0))));

    cg::Transform transform(cg::Location(180, 245, 2), cg::Rotation(0,-90,0));
    auto spectator = m_world.GetSpectator();
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
            m_world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    m_world.ApplySettings(defaultSettings);

    return 0;
}



