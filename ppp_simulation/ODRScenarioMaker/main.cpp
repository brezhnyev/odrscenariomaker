
#include <QtWidgets/QApplication>

#include "MainWindow.h"

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

#include <carla/client/Sensor.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include "../scenario.h"
#include "../Serializer.h"

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
#include <condition_variable>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <signal.h>

#include <eigen3/Eigen/Eigen>


namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool doStop;
static MainWindow * pmw;
static condition_variable cv;


void SaveImageToDisk(const csd::Image &image, int index, string type)
{
    using namespace carla::image;

    char buffer[9u];
    std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());

    auto filename = "_images/"s + type + "/" + to_string(index) + "/" + buffer + ".png";
    ImageIO::WriteView(filename, ImageView::MakeView(image));
    cv.notify_all();
}


void setDepthCams(std::vector<ShrdPtrActor> & cams, string blueprintName, string outname, cc::World & world, cc::ActorPtr ego)
{
    auto blueprint_library = world.GetBlueprintLibrary();
    // Find a camera blueprint.
    auto camera_bp = const_cast<cc::BlueprintLibrary::value_type*>(blueprint_library->Find(blueprintName));

    camera_bp->SetAttribute("image_size_x", "800");
    camera_bp->SetAttribute("image_size_y", "600");
    camera_bp->SetAttribute("fov", "90"); // set hard-coded 90 degrees, since 4x90=360 for depth cameras -> lidar

    for (int i = 0; i < 1; ++i)
    {
        auto camera_transform = cg::Transform{
            cg::Location{1,0,2},  // x, y, z.
            cg::Rotation{0.0f, i*90.0f, 0.0f}}; // pitch, yaw, roll.

        cams.push_back(world.SpawnActor(*camera_bp, camera_transform, ego.get()));
    }

    for (int i = 0; i < cams.size(); ++i) 
    {
        cc::Sensor* camera = static_cast<cc::Sensor*>(cams[i].get());
        // Register a callback to save images to disk.
        camera->Listen([i, outname](auto data) {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            SaveImageToDisk(*image, i, outname);
        });
    }
}


int play(Scenario & scenario)
{
    doStop = false;

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Town02");

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / 10); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    auto scenario_vehicles = scenario.children();
    const int number_of_vehicles = scenario_vehicles.size();
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
    vector<ShrdPtrActor> vehicles; vehicles.reserve(number_of_vehicles);

    cc::Vehicle::Control control;
    std::vector<ShrdPtrActor>               m_DepthCams;

    int i = 0;
    for (auto it = scenario_vehicles.begin(); it != scenario_vehicles.end(); ++it, ++i)
    {
        Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(it->second);
        auto blueprint = (*world.GetBlueprintLibrary()->Filter(scenario_vehicle.getName()))[0];
        if (blueprint.ContainsAttribute("color"))
        {
            auto &attribute = blueprint.GetAttribute("color");
            blueprint.SetAttribute("color", scenario_vehicle.colorToString());
        }
        // Spawn the vehicle.
        auto actor = world.TrySpawnActor(blueprint, spawn_points[i%number_of_vehicles]);
        if (!actor)
        {
            cout << "Failed to spawn actor ------" << endl;
            continue;
        }
        cout << "Spawned " << actor->GetDisplayId() << '\n';
        // Set the scenario start position:
        Waypath & waypath = *dynamic_cast<Waypath*>(scenario_vehicle.children().begin()->second);
        auto wit = waypath.children().begin();
        auto const wp1 = dynamic_cast<Waypoint*>(wit->second);
        ++wit;
        auto const wp2 = dynamic_cast<Waypoint*>(wit->second);
        Eigen::Vector3f dir = wp2->getPosition() - wp1->getPosition(); dir.y() *= -1; // Since Carla is LEFT handed - flip Y
        auto yaw = (atan2(dir.y(), dir.x()))*90/M_PI_2;
        cg::Transform transform(cg::Location(wp1->getPosition().x(), -wp1->getPosition().y(), wp1->getPosition().z()), cg::Rotation(0,yaw,0));
        actor->SetTransform(transform);
        auto vehicle = static_cast<cc::Vehicle*>(actor.get());
        control.throttle = 1.0f;
        vehicle->ApplyControl(control);
        vehicle->SetSimulatePhysics();
        vehicles.push_back(actor);

        if (0 == i)
            setDepthCams(m_DepthCams, "sensor.camera.depth", "depth", world, actor);
    }
    world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    // Set spectator trf by the first waypoint of the first vehicle:
    Vehicle & first_vehicle = *dynamic_cast<Vehicle*>(scenario_vehicles.begin()->second);
    auto spectator = world.GetSpectator();
    //transform.location -= 10.0f * transform.GetForwardVector();
    cg::Transform transform = vehicles[0]->GetTransform();
    transform.location.z += 30.0f;
    //transform.rotation.yaw += 180.0f;
    transform.rotation.pitch = -80.0f;
    spectator->SetTransform(transform);


    float SPEED = 10.0f;

    auto brake = [&](float strength, cc::Vehicle* vehicle)
    {
        auto ls = vehicle->GetLightState();
        auto speed =  vehicle->GetVelocity().Length();
        const float threshold = 0.05f;

        if (strength < threshold && speed < SPEED)
        {
            control.brake = 0.0f;
            control.throttle = exp(-speed/SPEED);
            ls = crpc::VehicleLightState::LightState::None;
        }
        else
        {
            strength = max(strength, threshold);
            control.brake = strength;
            control.throttle = 0.0f;
            ls = crpc::VehicleLightState::LightState::Brake;
        }

        vehicle->SetLightState(ls);
        vehicle->ApplyControl(control);
    };

    while (!doStop)
    {
        try
        {
            auto * actor = &vehicles[0];
            for (auto it = scenario_vehicles.begin(); it != scenario_vehicles.end(); ++it, ++actor)
            {
                auto vehicle = static_cast<cc::Vehicle*>((*actor).get());

                auto trf = vehicle->GetTransform();
                auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
                auto speed =  vehicle->GetVelocity().Length();

                Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(it->second);
                Waypath & waypath = *dynamic_cast<Waypath*>(scenario_vehicle.children().begin()->second);

                // Get the next waypoints in some distance away:
                Eigen::Vector3f peigen(trf.location.x, -trf.location.y, trf.location.z);
                if (!waypath.getNext(peigen))
                {
                    doStop = true;
                    break;
                }
                cg::Transform p(cg::Location(peigen.x(), -peigen.y(), peigen.z()));
                auto dir = (p.location - trf.location).MakeUnitVector();
                auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
                auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;

                control.steer = sign * arc.Length();
                vehicle->ApplyControl(control);
                // The stronger is the curvature the lower speed:
                float R = abs(3 * tan(M_PI_2 - control.steer)); // 3 is the ~length between axes
                float acc = speed*speed/R; // get centrifusual acceleration
                brake(0.01f*acc, vehicle);
                Actor * actor = dynamic_cast<Actor*>(scenario.children()[scenario_vehicle.getID()]);
                if (actor) actor->setTrf(trf.location.x, -trf.location.y,trf.location.z, -trf.rotation.yaw);
            }
            pmw->update();
            world.Tick(carla::time_duration(1s));

            mutex mtx;
            unique_lock<mutex> lk(mtx);
            cv.wait(lk);

        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    for (auto v : vehicles) v->Destroy();
    usleep(1e6);
}




int main(int argc, char ** argv)
{
    QApplication app(argc, argv);

    MainWindow mw; pmw = &mw;

    mw.show();

    return (app.exec());
}