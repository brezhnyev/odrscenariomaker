#include "Viewer.h"
#include "BasepolyExtractor.h"
#include "Osiexporter.h"
#include "odrparser/odrparser.h"

#include <qapplication.h>

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/detail/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/WalkerAIController.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>

#include <chrono>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <condition_variable>

#include <iostream>
#include <signal.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;
using namespace objl;
using namespace odr;
using namespace odr_1_5;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool isStopped;
condition_variable cv;
bool isObjParsed = false;
bool isXodrParsed = false;
bool isQtReady = false;

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


int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        cout << "Usage: " << argv[0] << " path/to/file.obj" << " path/to/file.xodr" << endl;
        return 0;
    }

    string mapName = "Munich02";
    if (argc > 3)
        mapName = argv[3];

    // Load the static parts:
    uint64_t id;
    Osiexporter osiex;
    Loader loader;
    vector<vector<Eigen::Vector2f>> baselines;

    thread t1([&](){
        loader.LoadFile(argv[1]);
        // export stationary
        for (auto && mesh : loader.LoadedMeshes)
        {
            string type = osiex.toValidType(mesh.MeshName);
            if (!type.empty())
            {
                vector<Eigen::Vector2f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
                vector<Eigen::Vector2f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
                // degenerated geometry case:
                if (convexBaseline.size() < 3)
                {
                    cout << mesh.MeshName << "   convex hull size less than 3! The shape is skipped!" << endl;
                    concaveBaseline = convexBaseline;
                }
                // concave cannot be smaller than convex (something went wrong in computing the concave form):
                if (concaveBaseline.size() < convexBaseline.size())
                {
                    cout << mesh.MeshName << "  concave hull is smaller than convex hull. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                // if computation of concave hull went into iternal loop and was broken by "convex.size > mesh.size" condition:
                if (concaveBaseline.size() > mesh.Vertices.size())
                {
                    cout << mesh.MeshName << "  concave hull is larger than original point cloud. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                // store the stationary object into OSI:
                vector<Eigen::Vector3f> v3d; v3d.reserve(mesh.Vertices.size());
                for (auto && v : mesh.Vertices) v3d.push_back(v.Position);
                osiex.addStaticObject(v3d, concaveBaseline, id, type);
                baselines.push_back(move(concaveBaseline));
            }
        }
        isObjParsed = true;
        cv.notify_all();
    }
    );


    vector<vector<Eigen::Vector2f>> centerlines, boundaries;
    thread t2([&](){
        // export road
        OpenDRIVEFile odr;
        loadFile(argv[2], odr);
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        isXodrParsed = true;
        cv.notify_all();
    }
    );


    // Carla set up:
    isStopped = false;
    signal(SIGINT, sighandler);

    mt19937_64 rng((random_device())());

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    //auto world = client.LoadWorld(mapName);
    auto world = client.GetWorld();

    auto traffic_manager = client.GetInstanceTM(8000); //KB: the port
    traffic_manager.SetGlobalDistanceToLeadingVehicle(2.0);
    traffic_manager.SetSynchronousMode(true);

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    uint32_t FPS = 30;
    crpc::EpisodeSettings wsettings(true, false, 1.0 / FPS); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);


    // Spawn Vehicles:
    const int number_of_vehicles = 50;
    auto blueprints = world.GetBlueprintLibrary()->Filter("vehicle.*");
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
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
        auto actor = world.TrySpawnActor(blueprint, transform);
        if (!actor) continue;
        // Finish and store the vehicle
        traffic_manager.SetPercentageIgnoreWalkers(actor, 0.0f);
        static_cast<cc::Vehicle*>(actor.get())->SetAutopilot(true);
        vehicles.push_back(actor);
        cout << "Spawned " << vehicles.back()->GetDisplayId() << '\n';
    }

    mutex mtx;
    unique_lock<std::mutex> lk(mtx);
    cv.wait(lk, [&]{return isObjParsed && isXodrParsed;});

    // Qt part should come after spawning, otherwise the application suspends
    Viewer * viewer = nullptr;
    thread t([&]()
    {
        QApplication application(argc, argv);

        viewer = new Viewer(move(baselines), move(centerlines), move(boundaries));
        viewer->setWindowTitle("Osi visualizer");
        viewer->show();

        isQtReady = true;

        cv.notify_all();

        application.exec();
    });
    cv.wait(lk, [&]{return isQtReady;});

    // ----------------


    uint64_t seconds;
    uint64_t nanos;

    while (!isStopped)
    {
        try
        {
            nanos += 1000000000.0/FPS;
            if (nanos > 1000000000)
            {
                seconds += 1;
                nanos = (uint64_t)(nanos)%1000000000;
            }
            osiex.setFrameTime(seconds, nanos);
            world.Tick(carla::time_duration(1s));

            // add to osi:
            vector<Eigen::Matrix4f> vizActors;
            osiex.updateMovingObjects(world.GetActors(), vizActors);
            osiex.writeFrame();
            // visuaization:
            viewer->updateMovingObjects(move(vizActors));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    t.join();

    //world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    for (auto && actor : *world.GetActors())
        actor->Destroy();

    return 0;
}
