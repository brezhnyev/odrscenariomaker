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
#include <omp.h>

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
bool isStaticParsed = false;
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

    float scale = 1.0f;
    if (argc > 4)
    {
        stringstream ss(argv[4]); // atof and stof are not reliable
        ss >> scale;
    }

    // Load the static parts:
    uint64_t id;
    Osiexporter osiex;
    Loader loader;
    vector<vector<Eigen::Vector2f>> centerlines, boundaries;
    vector<vector<Eigen::Vector2f>> baselines;

    thread t1([&](){

        cout << "Started parsing XODR file ..." << endl;
        // export road
        OpenDRIVEFile odr;
        loadFile(argv[2], odr);
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        cout << "Finished parsing XODR file ..." << endl;

        cout << "Started parsing OBJ file ..." << endl;
        loader.LoadFile(argv[1]);
        cout << "Started extracting base_poly ..." << endl;

        // export stationary
        mutex mtx1;
#pragma omp parallel for
        //for (auto && mesh : loader.LoadedMeshes)
        for (int i = 0; i < loader.LoadedMeshes.size(); ++i)
        {
            cout << std::this_thread::get_id() << endl;
            auto && mesh = loader.LoadedMeshes[i];
            string type = osiex.toValidType(mesh.MeshName);
            if (!type.empty())
            {
                vector<Eigen::Vector2f> convexBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, false);
                vector<Eigen::Vector2f> concaveBaseline = BasepolyExtractor::Obj2basepoly(mesh, loader, true);
                // degenerated geometry case:
                if (convexBaseline.size() < 3)
                {
                    cerr << mesh.MeshName << "   convex hull size less than 3! The shape is skipped!" << endl;
                    concaveBaseline = convexBaseline;
                }
                // concave cannot be smaller than convex (something went wrong in computing the concave form):
                if (concaveBaseline.size() < convexBaseline.size())
                {
                    cerr << mesh.MeshName << "  concave hull is smaller than convex hull. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                // if computation of concave hull went into iternal loop and was broken by "convex.size > mesh.size" condition:
                if (concaveBaseline.size() > mesh.Vertices.size())
                {
                    cerr << mesh.MeshName << "  concave hull is larger than original point cloud. Convex hull will be used." << endl;
                    concaveBaseline = convexBaseline;
                }
                vector<Eigen::Vector3f> v3d; v3d.reserve(mesh.Vertices.size());
                for (auto && v: concaveBaseline) v = v*scale;
                for (auto && v : mesh.Vertices) v3d.push_back(v.Position);
                // store the stationary object into OSI:
                {
                    lock_guard<mutex> lk(mtx1);
                    osiex.addStaticObject(v3d, concaveBaseline, id, type, scale);
                    baselines.push_back(move(concaveBaseline));
                }
            }
        }
        cout << "Finished extracting base_poly" << endl;
        isStaticParsed = true;
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

    auto world = client.LoadWorld(mapName);
    //auto world = client.GetWorld();

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

    // Spawn walkers:
    const int number_of_walkers = 50;
    vector<ShrdPtrActor> walkers; walkers.reserve(number_of_walkers);
    vector<ShrdPtrActor> wControllers; wControllers.reserve(number_of_walkers);

    auto w_bp = world.GetBlueprintLibrary()->Filter("walker.pedestrian.*"); // "Filter" returns BluePrintLibrary (i.e. wrapper about container of ActorBlueprints)

    vector<float> speeds; speeds.reserve(number_of_walkers);

    for (int i = 0; i < number_of_walkers; ++i)
    {
        auto location = world.GetRandomLocationFromNavigation();
        if (!location.has_value()) continue;
        auto walker_bp = RandomChoice(*w_bp, rng);
        if (walker_bp.ContainsAttribute("is_invincible")) walker_bp.SetAttribute("is_invincible", "false");
        auto walker = world.TrySpawnActor(walker_bp, location.value());
        if (!walker) continue;

        auto wc_bp = world.GetBlueprintLibrary()->Find("controller.ai.walker"); // "Find" returns pointer to the ActorBlueprint
        auto controller = world.TrySpawnActor(*wc_bp, cg::Transform(), walker.get());
        if (!controller) continue;

        // Store the walker and its controller
        walkers.push_back(walker);
        speeds.push_back(atof(walker_bp.GetAttribute("speed").GetRecommendedValues()[1].c_str()));
        wControllers.push_back(controller);
        cout << "Spawned " << walkers.back()->GetDisplayId() << '\n';
    }

    world.Tick(carla::time_duration(chrono::seconds(10)));

    for (int i = 0; i < wControllers.size(); ++i)
    {
        // KB: important! First Start then any settings like max speed.
        static_cast<cc::WalkerAIController*>(wControllers[i].get())->Start();
        static_cast<cc::WalkerAIController*>(wControllers[i].get())->SetMaxSpeed(speeds[i]);
    }

    world.SetPedestriansCrossFactor(0.0f);

    mutex mtx;
    unique_lock<std::mutex> lk(mtx);
    cv.wait(lk, [&]{return isStaticParsed;});

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
