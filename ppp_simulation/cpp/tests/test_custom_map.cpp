#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/detail/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/WalkerAIController.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>

#define PLATFORM_LINUX 1

#define WITH_EDITOR 0
#define WITH_ENGINE 0
#define WITH_UNREAL_DEVELOPER_TOOLS 0
#define WITH_PLUGIN_SUPPORT 0
#define IS_MONOLITHIC 0
#define IS_PROGRAM 0
#define UE_BUILD_DEBUG 1

#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

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
    isStopped = false;
    signal(SIGINT, sighandler);

    mt19937_64 rng((random_device())());

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Munich02");

    // Here we will set the vehicle, walker and point our spectator camera:
    cg::Transform transform, stransform;
    transform.location.x = -109;
    transform.location.y = 68;
    transform.location.z = 1;
    stransform = transform;
    stransform.location.x -= 20;
    stransform.location.z += 10;
    stransform.rotation.pitch = -30;

    // set the spectator:
    auto spectator = world.GetSpectator();
    spectator->SetTransform(stransform);

    auto traffic_manager = client.GetInstanceTM(8000); //KB: the port
    traffic_manager.SetGlobalDistanceToLeadingVehicle(2.0);
    traffic_manager.SetSynchronousMode(true);

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    //UGameplayStatics::OpenLevel(client.GetWorld(), "Munich01", true);

    // Spawn Vehicles:
    auto blueprints = world.GetBlueprintLibrary()->Filter("vehicle.toyota.prius");
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();

    auto blueprint = RandomChoice(*blueprints, rng);
    transform.rotation.yaw = -2.7;

    // Randomize the blueprint.
    if (blueprint.ContainsAttribute("color"))
    {
        auto &attribute = blueprint.GetAttribute("color");
        blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Spawn one vehicle ++++++++++++++++++++++++++++++:
    auto vactor = world.TrySpawnActor(blueprint, transform);
    cc::Vehicle* vehicle = static_cast<cc::Vehicle*>(vactor.get());
    // Finish and store the vehicle
    traffic_manager.SetPercentageIgnoreWalkers(vactor, 0.0f);
    vehicle->SetAutopilot(false);
    cout << "Spawned " << vehicle->GetDisplayId() << '\n';

    cc::Vehicle::Control control;
    vehicle->SetSimulatePhysics();

    // Spawn one walker ++++++++++++++++++++++++++++++:
    auto w_bp = world.GetBlueprintLibrary()->Filter("walker.pedestrian.0008"); // "Filter" returns BluePrintLibrary (i.e. wrapper about container of ActorBlueprints)
    auto wc_bp = world.GetBlueprintLibrary()->Find("controller.ai.walker"); // "Find" returns pointer to the ActorBlueprint

    transform.location.y = 72;
    auto walker_bp = RandomChoice(*w_bp, rng);
    if (walker_bp.ContainsAttribute("is_invincible")) walker_bp.SetAttribute("is_invincible", "false");
    auto wactor = world.TrySpawnActor(walker_bp, transform.location);
    auto wactorcontroller = world.TrySpawnActor(*wc_bp, cg::Transform(), wactor.get());

    // Store the walker and its controller
    cc::Walker * walker = static_cast<cc::Walker*>(wactor.get());
    float speed = atof(walker_bp.GetAttribute("speed").GetRecommendedValues()[1].c_str());
    cc::WalkerAIController* wController = static_cast<cc::WalkerAIController*>(wactorcontroller.get());
    cout << "Spawned " << walker->GetDisplayId() << '\n';

    world.Tick(carla::time_duration(chrono::seconds(10)));

    // KB: important! First Start then any settings like max speed.
    wController->Start();
    wController->SetMaxSpeed(speed);

    world.SetPedestriansCrossFactor(0.0f);

    while (!isStopped)
    {
        try
        {
            world.Tick(carla::time_duration(1s));
            control.throttle = 0.1f;
            vehicle->ApplyControl(control);
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    vehicle->Destroy();
    walker->Destroy();

    return 0;
}
