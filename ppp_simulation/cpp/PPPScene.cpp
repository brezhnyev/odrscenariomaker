#include "PPPScene.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
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
    return range[dist(std::forward<RNG>(generator))];
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
static void SaveSemSegImageToDisk(const csd::Image &image)
{
    using namespace carla::image;

    char buffer[9u];
    std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
    auto filename = "_images/"s + buffer + ".png";

    auto view = ImageView::MakeColorConvertedView(
        ImageView::MakeView(image),
        ColorConverter::CityScapesPalette());
    ImageIO::WriteView(filename, view);
}

static auto ParseArguments(int argc, const char *argv[])
{
    EXPECT_TRUE((argc == 1u) || (argc == 3u));
    using ResultType = std::tuple<std::string, uint16_t>;
    return argc == 3u ? ResultType{argv[1u], std::stoi(argv[2u])} : ResultType{"localhost", 2000u};
}

PPPScene::PPPScene(string confname) : m_doRun(false)
{
    YAML::Node config = YAML::LoadFile(confname.c_str());
    string host = config["common"]["host"].as<string>();
    uint16_t port = config["common"]["port"].as<int>();

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(10s);

    std::cout << "Client API version : " << client.GetClientVersion() << '\n';
    std::cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto town_name = config["common"]["town"].as<string>();
    std::cout << "Loading world: " << town_name << std::endl;
    m_world = new cc::World(move(client.LoadWorld(town_name)));

    // Get a random vehicle blueprint.
    auto blueprint_library = m_world->GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    auto blueprint = RandomChoice(*vehicles, rng);

    // Randomize the blueprint.
    if (blueprint.ContainsAttribute("color"))
    {
        auto &attribute = blueprint.GetAttribute("color");
        blueprint.SetAttribute(
            "color",
            RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    // Find a valid spawn point.
    auto map = m_world->GetMap();
    auto transform = RandomChoice(map->GetRecommendedSpawnPoints(), rng);

    // Spawn the vehicle.
    m_actor = m_world->SpawnActor(blueprint, transform);
    std::cout << "Spawned " << m_actor->GetDisplayId() << '\n';
    auto vehicle = static_cast<cc::Vehicle*>(m_actor.get());

    // Apply control to vehicle.
    cc::Vehicle::Control control;
    control.throttle = 1.0f;
    vehicle->ApplyControl(control);
    vehicle->SetAutopilot(true);

    // Move spectator so we can see the vehicle from the simulator window.
    auto spectator = m_world->GetSpectator();
    transform.location += 7.0f * transform.GetForwardVector();
    transform.location.z += 2.0f;
    transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -15.0f;
    spectator->SetTransform(transform);

    // Find a camera blueprint.
    auto camera_bp = blueprint_library->Find("sensor.camera.semantic_segmentation");
    EXPECT_TRUE(camera_bp != nullptr);

    // Spawn a camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{-5.5f, 0.0f, 2.8f},   // x, y, z.
        cg::Rotation{-15.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    m_cam_actor = m_world->SpawnActor(*camera_bp, camera_transform, m_actor.get());

    // Synchronous mode:
    int fps = config["common"]["fps"].as<int>();
    m_defaultSettings = m_world->GetSettings();
    EpisodeSettings wsettings(true, false, 1.0 / fps); // (doRender, isSynchrone, interval)
    m_world->ApplySettings(wsettings);
}

void PPPScene::start()
{
    m_thread = new thread([this](){
        // Asynchronous mode:
        //std::this_thread::sleep_for(100s);

        m_doRun = true;

        auto camera = static_cast<cc::Sensor*>(m_cam_actor.get());

        // Register a callback to save images to disk.
        camera->Listen([](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            EXPECT_TRUE(image != nullptr);
            SaveSemSegImageToDisk(*image);
        });

        // Synchronous mode:
        while (m_doRun)
        {
            m_world->Tick(carla::time_duration(std::chrono::milliseconds(1000)));
        }

        camera->Stop();
        usleep(1e6); // wait for all callback to finish
    });
}

void PPPScene::setSensors(string confname)
{

}

void PPPScene::stop()
{
    m_doRun = false;
    cout << "Waiting to stop the client" << endl;
    m_thread->join();

    // Remove actors from the simulation.
    m_cam_actor.get()->Destroy();
    m_actor.get()->Destroy();
    m_world->ApplySettings(m_defaultSettings); // reset again to the asynchronous mode
    delete m_world;
    delete m_thread;
    std::cout << "Actors destroyed." << std::endl;
}