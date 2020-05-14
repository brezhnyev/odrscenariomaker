#include "PPPScene.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/client/WalkerAIController.h>

#include <yaml-cpp/yaml.h>

#include <string>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <string>
#include <chrono>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace carla::rpc;
using namespace std;

#define EXPECT_TRUE(pred)                \
    if (!(pred))                         \
    {                                    \
        throw std::runtime_error(#pred); \
    }

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator)
{
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))]; // KB: this can fail for map
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
void PPPScene::SaveImageToDisk(const csd::Image &image, int index, string type)
{
    using namespace carla::image;

    char buffer[9u];
    std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());

    if (type == "rgb_ss" || type == "depth_ss")
    {
        auto filename = "_images/"s + type + "_color/" + to_string(index) + "/" + buffer + ".png";
        auto view = ImageView::MakeColorConvertedView(
            ImageView::MakeView(image),
            ColorConverter::CityScapesPalette());
        ImageIO::WriteView(filename, view);
    }
    auto filename = "_images/"s + type + "/" + to_string(index) + "/" + buffer + ".png";
    ImageIO::WriteView(filename, ImageView::MakeView(image));
}


PPPScene::PPPScene(string confname) : m_doRun(false), m_isInitialized(false)
{
    try
    {
        YAML::Node config = YAML::LoadFile(confname.c_str());

        string host = config["common"]["host"].as<string>();
        uint16_t port = config["common"]["port"].as<int>();
        m_timeout = config["common"]["timeout"].as<int>();

        std::mt19937_64 rng((std::random_device())());

        auto client = cc::Client(host, port);
        client.SetTimeout(carla::time_duration(std::chrono::seconds(m_timeout)));

        std::cout << "Client API version : " << client.GetClientVersion() << '\n';
        std::cout << "Server API version : " << client.GetServerVersion() << '\n';

        auto town_name = config["common"]["town"].as<string>();
        std::cout << "Loading world: " << town_name << std::endl;
        m_world = boost::make_shared<cc::World>(client.LoadWorld(town_name));

        auto traffic_manager = client.GetInstanceTM(); //KB: the port
        traffic_manager.SetGlobalDistanceToLeadingVehicle(2.0);
        traffic_manager.SetSynchronousMode(true);

        // Spawn vehicles:
        spawnVehicles(confname);
        spawnWalkers(confname);

        // // Set sensors:
        // setRGBCams(confname, m_RGBCams, "sensor.camera.rgb", "rgb");
        // setRGBCams(confname, m_RGBCamsSS, "sensor.camera.semantic_segmentation", "rgb_ss");
        // setDepthCams(confname, m_DepthCams, "sensor.camera.depth", "depth");
        // setDepthCams(confname, m_DepthCamsSS, "sensor.camera.semantic_segmentation", "depth_ss");

        // Set weather:
        setWeather(confname);

        // Synchronous mode:
        int fps = config["common"]["fps"].as<int>();
        m_defaultSettings = m_world->GetSettings();
        EpisodeSettings wsettings(true, false, 1.0 / fps); // (synchrone, noRender, interval)
        m_world->ApplySettings(wsettings);

        m_isInitialized = true;
    }
    catch (const carla::client::TimeoutException &e)
    {
        std::cout << '\n' << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "\nException: " << e.what() << std::endl;
    }
}

void PPPScene::start()
{
    m_thread = new thread([this](){
        // Asynchronous mode:
        //std::this_thread::sleep_for(100s);

        m_doRun = true;

        // Synchronous mode:

        while (m_doRun)
        {
            try
            {
                m_world->Tick(carla::time_duration(std::chrono::seconds(m_timeout)));
            }
            catch (const std::exception &e)
            {
                std::cout << "\nException: " << e.what() << std::endl;
                continue;
            }
        }

        for (auto s : m_RGBCams) static_cast<cc::Sensor*>(s.get())->Stop();
        for (auto s : m_RGBCamsSS) static_cast<cc::Sensor*>(s.get())->Stop();
        for (auto s : m_DepthCams) static_cast<cc::Sensor*>(s.get())->Stop();
        for (auto s : m_DepthCamsSS) static_cast<cc::Sensor*>(s.get())->Stop();

        for (auto s : m_wControllers) static_cast<cc::WalkerAIController*>(s.get())->Stop();
        
        cout << "Waiting 10 seconds to stop the client (ensure last callbacks are processed)." << endl;
        usleep(1e7); // wait for all callback to finish
    });
}

void PPPScene::setRGBCams(string confname, std::vector<ShrdPtrActor> & cams, string blueprintName, string outname)
{
    auto blueprint_library = m_world->GetBlueprintLibrary();
    // Find a camera blueprint.
    auto camera_bp = const_cast<cc::BlueprintLibrary::value_type*>(blueprint_library->Find(blueprintName));
    EXPECT_TRUE(camera_bp != nullptr);

    YAML::Node config = YAML::LoadFile(confname.c_str());
    camera_bp->SetAttribute("image_size_x", to_string(config["common"]["width"].as<int>()));
    camera_bp->SetAttribute("image_size_y", to_string(config["common"]["height"].as<int>()));
    camera_bp->SetAttribute("fov", to_string(config["common"]["fov"].as<int>()));

    auto yamlcams = config["sensors"]["cameras"];
    for (auto && c : yamlcams)
    {
        auto camera_transform = cg::Transform{
            cg::Location{c["x"].as<float>(), c["y"].as<float>(), c["z"].as<float>()},        // x, y, z.
            cg::Rotation{c["pitch"].as<float>(), c["yaw"].as<float>(), c["roll"].as<float>()}}; // pitch, yaw, roll.

        cams.push_back(m_world->SpawnActor(*camera_bp, camera_transform, m_actor.get()));
    }

    for (int i = 0; i < cams.size(); ++i)
    {
        auto camera = static_cast<cc::Sensor*>(cams[i].get());
        // Register a callback to save images to disk.
        camera->Listen([this, i, outname](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            EXPECT_TRUE(image != nullptr);
            SaveImageToDisk(*image, i, outname);
        });
    }
}

void PPPScene::setDepthCams(string confname, std::vector<ShrdPtrActor> & cams, string blueprintName, string outname)
{
    auto blueprint_library = m_world->GetBlueprintLibrary();
    // Find a camera blueprint.
    auto camera_bp = const_cast<cc::BlueprintLibrary::value_type*>(blueprint_library->Find(blueprintName));
    EXPECT_TRUE(camera_bp != nullptr);

    YAML::Node config = YAML::LoadFile(confname.c_str());
    camera_bp->SetAttribute("image_size_x", to_string(config["common"]["width"].as<int>()));
    camera_bp->SetAttribute("image_size_y", to_string(config["common"]["height"].as<int>()));
    camera_bp->SetAttribute("fov", "90"); // set hard-coded 90 degrees, since 4x90=360 for depth cameras -> lidar

    auto lidarPos = config["sensors"]["lidar"];
    for (int i = 0; i < 4; ++i)
    {
        auto camera_transform = cg::Transform{
            cg::Location{lidarPos["x"].as<float>(), lidarPos["y"].as<float>(), lidarPos["z"].as<float>()},  // x, y, z.
            cg::Rotation{0.0f, i*90.0f, 0.0f}}; // pitch, yaw, roll.

        cams.push_back(m_world->SpawnActor(*camera_bp, camera_transform, m_actor.get()));
    }

    for (int i = 0; i < cams.size(); ++i)
    {
        auto camera = static_cast<cc::Sensor*>(cams[i].get());
        // Register a callback to save images to disk.
        camera->Listen([this, i, outname](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            EXPECT_TRUE(image != nullptr);
            SaveImageToDisk(*image, i, outname);
        });
    }
}

void PPPScene::stop()
{
    m_doRun = false;
    m_thread->join();

    try
    {
        // Remove vehicles and walkers:
        for (auto && c : m_vehicles) c.get()->Destroy();
        for (auto && c : m_wControllers) c.get()->Destroy();    
        for (auto && c : m_walkers) c.get()->Destroy();

        // Remove sensors from the simulation.
        for (auto && c : m_RGBCams) c.get()->Destroy();
        for (auto && c : m_RGBCamsSS) c.get()->Destroy();
        for (auto && c : m_DepthCams) c.get()->Destroy();
        for (auto && c : m_DepthCamsSS) c.get()->Destroy();
    }
    catch(...)
    {
        cout << "Exception thrown during destroying objects. Ignored." << endl;
    }
    m_actor.get()->Destroy();
    m_world->ApplySettings(m_defaultSettings); // reset again to the asynchronous mode
    delete m_thread;
    std::cout << "Actors destroyed." << std::endl;
}

void PPPScene::setWeather(string confname)
{
    YAML::Node config = YAML::LoadFile(confname.c_str());
    // Set weather:
    auto wpreset = config["weather"]["_preset"];
    if (!wpreset.IsNull())
    {
        if (wpreset.as<string>() == "Default") m_world->SetWeather(WeatherParameters::Default);
        if (wpreset.as<string>() == "ClearNoon") m_world->SetWeather(WeatherParameters::ClearNoon);
        if (wpreset.as<string>() == "CloudyNoon") m_world->SetWeather(WeatherParameters::CloudyNoon);
        if (wpreset.as<string>() == "WetNoon") m_world->SetWeather(WeatherParameters::WetNoon);
        if (wpreset.as<string>() == "WetCloudyNoon") m_world->SetWeather(WeatherParameters::WetCloudyNoon);
        if (wpreset.as<string>() == "MidRainyNoon") m_world->SetWeather(WeatherParameters::MidRainyNoon);
        if (wpreset.as<string>() == "HardRainNoon") m_world->SetWeather(WeatherParameters::HardRainNoon);
        if (wpreset.as<string>() == "SoftRainNoon") m_world->SetWeather(WeatherParameters::SoftRainNoon);
        if (wpreset.as<string>() == "ClearSunset") m_world->SetWeather(WeatherParameters::ClearSunset);
        if (wpreset.as<string>() == "CloudySunset") m_world->SetWeather(WeatherParameters::CloudySunset);
        if (wpreset.as<string>() == "WetSunset") m_world->SetWeather(WeatherParameters::WetSunset);
        if (wpreset.as<string>() == "WetCloudySunset") m_world->SetWeather(WeatherParameters::WetCloudySunset);
        if (wpreset.as<string>() == "MidRainSunset") m_world->SetWeather(WeatherParameters::MidRainSunset);
        if (wpreset.as<string>() == "SoftRainSunset") m_world->SetWeather(WeatherParameters::SoftRainSunset);
    }
    else
    {
        auto weather = WeatherParameters
        (
            config["weather"]["cloudness"].as<float>(),
            config["weather"]["precipitation"].as<float>(),
            config["weather"]["precipitation_deposits"].as<float>(),
            config["weather"]["wind_intensity"].as<float>(),
            config["weather"]["sun_azimuth_angle"].as<float>(),
            config["weather"]["sun_altitude_angle"].as<float>(),
            config["weather"]["fog_density"].as<float>(),
            config["weather"]["fog_distance"].as<float>(),
            config["weather"]["wetness"].as<float>()
        );
        m_world->SetWeather(weather);
    }
}

void PPPScene::spawnVehicles(string confname)
{
    YAML::Node config = YAML::LoadFile(confname.c_str());
    string filter = config["spawner"]["vehicles"]["filter"].as<string>();
    auto blueprints = m_world->GetBlueprintLibrary()->Filter(filter);
    auto spawn_points = m_world->GetMap()->GetRecommendedSpawnPoints();
    int number_of_vehicles = config["spawner"]["vehicles"]["number"].as<int>();
    if (spawn_points.size() < number_of_vehicles)
        cout << "Number of requested vehicle is larger than the maximum, that can be generated (" << spawn_points.size() << ")" << endl;
    number_of_vehicles = min(number_of_vehicles, (int)spawn_points.size());
    cout << "The " << number_of_vehicles << " vehicles will be spawned." << endl;
    m_vehicles.reserve(number_of_vehicles);

    std::mt19937_64 rng((std::random_device())());

    int fails = 0;
    const int MAXFAILS = 10;
    for (int i = 0; i < number_of_vehicles;)
    {
        if (fails > MAXFAILS)
        {
            cout << "Too many fails spawning vehicles. Breaking spawning." << endl;
            break;
        }
        auto blueprint = RandomChoice(*blueprints, rng);
        // Find a valid spawn point.
        auto transform = RandomChoice(spawn_points, rng);

        // Randomize the blueprint.
        if (blueprint.ContainsAttribute("color"))
        {
            auto &attribute = blueprint.GetAttribute("color");
            blueprint.SetAttribute("color", RandomChoice(attribute.GetRecommendedValues(), rng));
            blueprint.SetAttribute("role_name", "autopilot");
        }

        // Spawn the vehicle.
        auto actor = m_world->TrySpawnActor(blueprint, transform);
        if (!actor)
        {
            ++fails;
            cout << "Failed to spawn vehicle. Lets try again." << endl;
            continue;
        }
        // Finish and store the vehicle
        m_vehicles.push_back(actor);
        auto vehicle = static_cast<cc::Vehicle*>(actor.get());
        // Apply control to vehicle.
        cc::Vehicle::Control control;
        control.throttle = 1.0f;
        vehicle->ApplyControl(control);
        vehicle->SetAutopilot(true);
        ++i;
        fails = 0;
        std::cout << "Spawned " << m_vehicles.back()->GetDisplayId() << '\n';
    }
    if (number_of_vehicles)
    {
        m_actor = m_vehicles[0];
        // Move spectator so we can see the vehicle from the simulator window.
        auto spectator = m_world->GetSpectator();
        auto transform = m_actor->GetTransform();
        transform.location += 7.0f * transform.GetForwardVector();
        transform.location.z += 2.0f;
        transform.rotation.yaw += 180.0f;
        transform.rotation.pitch = -15.0f;
        spectator->SetTransform(transform);
    }
}

void PPPScene::spawnWalkers(string confname)
{
    YAML::Node config = YAML::LoadFile(confname.c_str());
    int number_of_walkers = config["spawner"]["walkers"]["number"].as<int>();

    int percentagePedestriansRunning = config["spawner"]["walkers"]["running"].as<int>();       // how many pedestrians will run
    int percentagePedestriansCrossing = config["spawner"]["walkers"]["crossingRoad"].as<int>(); // how many pedestrians will walk through the road

    // Spawn the walker object.
    m_walkers.reserve(number_of_walkers);
    string filter = config["spawner"]["walkers"]["filter"].as<string>();
    auto w_bp = m_world->GetBlueprintLibrary()->Filter(filter); // "Filter" returns BluePrintLibrary (i.e. wrapper about container of ActorBlueprints)
    auto wc_bp = m_world->GetBlueprintLibrary()->Find("controller.ai.walker"); // "Find" returns pointer to the ActorBlueprint

    std::mt19937_64 rng((std::random_device())());

    vector<float> speeds; speeds.reserve(number_of_walkers);

    int fails = 0;
    const int MAXFAILS = 10;
    for (int i = 0; i < number_of_walkers; )
    {
        if (fails > MAXFAILS)
        {
            cout << "Too many fails spawning walkers. Breaking spawning." << endl;
            break;
        }
        auto location = m_world->GetRandomLocationFromNavigation();
        if (!location.has_value())
        {
            ++fails;
            continue;
        }
        auto walker_bp = RandomChoice(*w_bp, rng);
        //if (walker_bp.ContainsAttribute("is_invincible")) walker_bp.SetAttribute("is_invincible", "false");
        walker_bp.SetAttribute("is_invincible", "false");
        auto walker = m_world->TrySpawnActor(walker_bp, location.value());
        if (walker)
        {
            auto controller = m_world->TrySpawnActor(*wc_bp, cg::Transform(), walker.get());
            if (!controller)
            {
                walker.get()->Destroy();
                ++fails;
                cout << "Failed to spawn walker. Lets try again." << endl;
                continue;
            }
            else
            {
                // Store the walker and its controller
                m_walkers.push_back(walker);
                speeds.push_back(atof(walker_bp.GetAttribute("speed").GetRecommendedValues()[1].c_str()));
                m_wControllers.push_back(controller);
                ++i;
                fails = 0;
                std::cout << "Spawned " << m_walkers.back()->GetDisplayId() << '\n';
            }
        }
        else
        {
            ++fails;
            cout << "Failed to spawn walker. Lets try again." << endl;
            continue;
        }
    }

    for (int i = 0; i < m_wControllers.size(); ++i)
    {
        // KB: important! First Start then any settings like max speed.
        static_cast<cc::WalkerAIController*>(m_wControllers[i].get())->Start();
        static_cast<cc::WalkerAIController*>(m_wControllers[i].get())->SetMaxSpeed(speeds[i]);
    }
}