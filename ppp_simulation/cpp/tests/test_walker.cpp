#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/detail/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/WalkerAIController.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>
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

    auto m_world = client.LoadWorld("Town02");

    auto traffic_manager = client.GetInstanceTM(8000); //KB: the port
    traffic_manager.SetGlobalDistanceToLeadingVehicle(2.0);
    traffic_manager.SetSynchronousMode(true);

    // Synchronous mode:
    auto defaultSettings = m_world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 30); // (synchrone, noRender, interval)
    m_world.ApplySettings(wsettings);
    m_world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn walkers:
    const int number_of_walkers = 1;
    vector<ShrdPtrActor> walkers; walkers.reserve(number_of_walkers);
    vector<ShrdPtrActor> wControllers; wControllers.reserve(number_of_walkers);

    auto w_bp = m_world.GetBlueprintLibrary()->Filter("walker.pedestrian.0004"); // "Filter" returns BluePrintLibrary (i.e. wrapper about container of ActorBlueprints)
    auto wc_bp = m_world.GetBlueprintLibrary()->Find("controller.ai.walker"); // "Find" returns pointer to the ActorBlueprint

    vector<float> speeds; speeds.reserve(number_of_walkers);

    for (int i = 0; i < number_of_walkers;)
    {
        auto walker_bp = RandomChoice(*w_bp, rng);
        if (walker_bp.ContainsAttribute("is_invincible")) walker_bp.SetAttribute("is_invincible", "false");
        auto walker = m_world.TrySpawnActor(walker_bp, cg::Location(0,0,0));
        if (!walker) continue;
        ++i;

        // Store the walker and its controller
        walkers.push_back(walker);
        speeds.push_back(atof(walker_bp.GetAttribute("speed").GetRecommendedValues()[1].c_str()));
        cout << "Spawned " << walkers.back()->GetDisplayId() << '\n';
    }

    m_world.Tick(carla::time_duration(chrono::seconds(10)));

    cg::Transform transform(cg::Location(180, 245, 2), cg::Rotation(0,-90,0));
    walkers[0]->SetTransform(transform);
    auto spectator = m_world.GetSpectator();
    transform.location += 15.0f * transform.GetForwardVector();
    transform.location.z += 5.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -45.0f;
    spectator->SetTransform(transform);

    auto walker = static_cast<cc::Walker*>(walkers[0].get());
    auto wcontrol = walker->GetWalkerControl();

    m_world.SetPedestriansCrossFactor(0.0f);

    while (!isStopped)
    {
        try
        {
            // crpc::WalkerBoneControl wbc;
            // crpc::BoneTransformData btd = make_pair<std::string, cg::Transform>("crl_hand__R", cg::Transform(cg::Location(0,0,0), cg::Rotation(0,0,90)));
            // wbc.bone_transforms.push_back(btd);
            // walker->ApplyControl(wbc);
            
            wcontrol.speed = 1.5f;
            wcontrol.direction = cg::Vector3D(0,-1,0);
            walker->ApplyControl(wcontrol);
            m_world.Tick(carla::time_duration(1s));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    m_world.ApplySettings(defaultSettings);
    for (auto w : walkers)  w->Destroy();

    return 0;
}
