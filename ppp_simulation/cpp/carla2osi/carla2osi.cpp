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

    // first start the Qt part in a thread and loader (this will make loading in parallel to Carla Map computation):
    Osiexporter osiex;
    Loader loader;
    Viewer * viewer = nullptr;

    thread t([&]()
    {
        QApplication application(argc, argv);

        viewer = new Viewer();
        viewer->setWindowTitle("Osi visualizer");
        viewer->show();
        loader.LoadFile(argv[1]);
        uint64_t id;

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
                // visualize
                viewer->addDataStatic(move(concaveBaseline));
            }
        }

        // export road
        OpenDRIVEFile odr;
        loadFile(argv[2], odr);
        vector<vector<Eigen::Vector2f>> centerlines, boundaries;
        osiex.addRoads(*odr.OpenDRIVE1_5, id, centerlines, boundaries);
        // visualize
        viewer->updateDataRoads(move(centerlines), move(boundaries));

        application.exec();
    });
    // ----------------

    // Carla set up:
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
    uint32_t FPS = 30;
    crpc::EpisodeSettings wsettings(true, false, 1.0 / FPS); // (synchrone, noRender, interval)
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

    vector<cc::Actor*> actors;
    actors.push_back(vehicle);
    actors.push_back(walker);

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

            control.throttle = 0.2f;
            vehicle->ApplyControl(control);
            // add to osi:
            vector<Eigen::Matrix4f> vizActors;
            osiex.updateMovingObjects(actors, vizActors);
            osiex.writeFrame();
            // visuaization:
            viewer->updateMovingObjects(move(vizActors));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    t.join();

    //world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    vehicle->Destroy();
    walker->Destroy();

    return 0;
}
