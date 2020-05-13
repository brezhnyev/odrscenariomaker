#include "PPPScene.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/rpc/EpisodeSettings.h>

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


PPPScene::PPPScene(string confname) : m_doRun(false)
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

    // Set sensors:
    setRGBCams(confname, m_RGBCams, "sensor.camera.rgb", "rgb");
    setRGBCams(confname, m_RGBCamsSS, "sensor.camera.semantic_segmentation", "rgb_ss");
    setDepthCams(confname, m_DepthCams, "sensor.camera.depth", "depth");
    setDepthCams(confname, m_DepthCamsSS, "sensor.camera.semantic_segmentation", "depth_ss");

    // Set weather:
    setWeather(confname);


    // Synchronous mode:
    int fps = config["common"]["fps"].as<int>();
    m_defaultSettings = m_world->GetSettings();
    EpisodeSettings wsettings(true, false, 1.0 / fps); // (synchrone, noRender, interval)
    m_world->ApplySettings(wsettings);
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
            m_world->Tick(carla::time_duration(std::chrono::seconds(m_timeout)));
        }

        for (auto && c : m_RGBCams) static_cast<cc::Sensor*>(c.get())->Stop();
        for (auto && c : m_RGBCamsSS) static_cast<cc::Sensor*>(c.get())->Stop();
        for (auto && c : m_DepthCams) static_cast<cc::Sensor*>(c.get())->Stop();
        for (auto && c : m_DepthCamsSS) static_cast<cc::Sensor*>(c.get())->Stop();
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

    // Remove actors from the simulation.
    for (auto && c : m_RGBCams) c.get()->Destroy();
    for (auto && c : m_RGBCamsSS) c.get()->Destroy();
    for (auto && c : m_DepthCams) c.get()->Destroy();
    for (auto && c : m_DepthCamsSS) c.get()->Destroy();
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
    string vfilter = config["spawner"]["filterv"].as<string>();
    auto blueprints = m_world->GetBlueprintLibrary()->Filter(vfilter);
    auto spawn_points = m_world->GetMap()->GetRecommendedSpawnPoints();
    int number_of_vehicles = config["spawner"]["number_of_vehicles"].as<int>();
    if (spawn_points.size() < number_of_vehicles)
        cout << "Number of requested vehicle is larger than the maximum, that can be generated (" << spawn_points.size() << ")" << endl;
    number_of_vehicles = min(number_of_vehicles, (int)spawn_points.size());
    cout << "The " << number_of_vehicles << " vehicles will be spawned." << endl;
    m_vehicles.reserve(number_of_vehicles);

    std::mt19937_64 rng((std::random_device())());

    for (int i = 0; i < number_of_vehicles; ++i)
    {
        try
        {
            auto blueprint = RandomChoice(*blueprints, rng);
            // Find a valid spawn point.
            auto transform = RandomChoice(spawn_points, rng);

            // Randomize the blueprint.
            if (blueprint.ContainsAttribute("color"))
            {
                auto &attribute = blueprint.GetAttribute("color");
                blueprint.SetAttribute(
                    "color",
                    RandomChoice(attribute.GetRecommendedValues(), rng));
            }

            // Spawn the vehicle.
            m_vehicles.push_back(m_world->SpawnActor(blueprint, transform));
            auto vehicle = static_cast<cc::Vehicle*>(m_vehicles.back().get());

            // Apply control to vehicle.
            cc::Vehicle::Control control;
            control.throttle = 1.0f;
            vehicle->ApplyControl(control);
            vehicle->SetAutopilot(true);

            std::cout << "Spawned " << m_vehicles.back()->GetDisplayId() << '\n';
        }
        catch (...) {} // KB: ignore the collision exception
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