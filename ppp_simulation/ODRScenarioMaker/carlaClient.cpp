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
#include <carla/sensor/data/IMUMeasurement.h>

#include "./scenario.h"

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
#include "MainWindow.h"


namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

typedef carla::SharedPtr<cc::Actor> ShrdPtrActor;

extern int playStatus;
extern condition_variable playCondVar;
extern mutex playCondVarMtx;
extern Matrix4f camTrf;
extern MainWindow * mw;
static int FPS = 30;
static bool realtime_playback = true;
//extern vector<QLabel*> camera_widgets; // need to be extern to be created in the parent thread


void play(Scenario & scenario)
{
    playStatus = 2; // 0 stop, 1 pause, 2 play
    camTrf.setIdentity();

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = scenario.getTownName().empty() ? client.GetWorld() : client.LoadWorld(scenario.getTownName());

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / FPS); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    auto weather = world.GetWeather();
    weather.fog_density = 0.0f; // higher saturation
    weather.mie_scattering_scale = 0.0f; // better sky contrast
    weather.sun_altitude_angle = 80.0f;
    weather.cloudiness = 10.0f;
    world.SetWeather(weather);

    vector<ShrdPtrActor> vehicles;
    std::vector<ShrdPtrActor> cameras;
    ShrdPtrActor actor;

    for (auto && child : scenario.children())
    {
        Vehicle * scenario_vehicle = dynamic_cast<Vehicle*>(child.second);
        if (scenario_vehicle) // can be also ex. Sensor (Camera or Lidar) or Walker 
        {
            auto blueprint = (*world.GetBlueprintLibrary()->Filter(scenario_vehicle->getName()))[0];
            if (blueprint.ContainsAttribute("color"))
            {
                auto &attribute = blueprint.GetAttribute("color");
                blueprint.SetAttribute("color", scenario_vehicle->colorToString());
            }
            // Set the scenario start position:
            for (auto && child : scenario_vehicle->children())
            {
                Waypath * waypath = dynamic_cast<Waypath*>(child.second);
                // KB: here we need to think if we really need multiple waypaths for a vehicle
                if (waypath && !waypath->children().empty())
                {
                    waypath->updateSmoothPath();
                    Eigen::Vector3f dir = waypath->getStartingDirection();
                    Eigen::Vector3f pos = waypath->getStartingPosition();
                    auto yaw = (atan2(-dir.y(), dir.x()))*90/M_PI_2; // Since Carla is LEFT handed - flip Y
                    cg::Transform transform(cg::Location(pos.x(), -pos.y(), pos.z()+0.5), cg::Rotation(0,yaw,0)); // Since Carla is LEFT handed - flip Y
                    // Spawn the vehicle.
                    actor = world.TrySpawnActor(blueprint, transform);
                    if (!actor)
                    {
                        cout << "Failed to spawn actor ------" << endl;
                        continue;
                    }
                    cout << "Spawned " << actor->GetDisplayId() << '\n';
                    auto vehicle = dynamic_cast<cc::Vehicle*>(actor.get());
                    vehicle->SetSimulatePhysics();
                    vehicles.push_back(actor);
                }
                Camera * camera = dynamic_cast<Camera*>(child.second);
                if (camera)
                {
                    auto blueprint_library = world.GetBlueprintLibrary();
                    // Find a camera blueprint.
                    auto camera_bp = const_cast<cc::BlueprintLibrary::value_type*>(blueprint_library->Find("sensor.camera.rgb"));

                    camera_bp->SetAttribute("image_size_x", "1280");
                    camera_bp->SetAttribute("image_size_y", "720");
                    camera_bp->SetAttribute("fov", to_string(camera->getFOV()));

                    auto camera_transform = cg::Transform{
                        cg::Location{camera->getPos().x(), camera->getPos().y(), camera->getPos().z()},        // x, y, z.
                        cg::Rotation{camera->getOri().y(), camera->getOri().z(), camera->getOri().x()}}; // pitch, yaw, roll.

                    cameras.push_back(world.SpawnActor(*camera_bp, camera_transform, actor.get()));
                    camera->getCamWidget()->resize(1280, 720);
                    camera->getCamWidget()->show();

                    // Register a callback to save images to disk.
                    ((cc::Sensor*)cameras.back().get())->Listen([camera](auto data)
                    {
                        auto image = boost::static_pointer_cast<csd::Image>(data);
                        QPixmap backBuffer = QPixmap::fromImage(QImage((unsigned char*)image->data(), image->GetWidth(), image->GetHeight(), QImage::Format_RGBX8888).rgbSwapped());
                        camera->getCamWidget()->setPixmap(backBuffer.scaled(camera->getCamWidget()->size(), Qt::KeepAspectRatio));
                        camera->getCamWidget()->update();
                    });
                }
            }
        }
    }
    world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    auto applyControl = [&](float accLat, float accLon, cc::Vehicle* vehicle, float targetSpeed, float steer)
    {
        auto ls = vehicle->GetLightState();
        auto speed =  vehicle->GetVelocity().Length();
        cc::Vehicle::Control control;
        control.steer = steer;
        if (accLon >= 0)
        {
            control.brake = 0.0f;
            control.throttle = abs(accLon);
            ls = crpc::VehicleLightState::LightState::None;
        }
        if (accLon < 0 || !speed)
        {
            control.brake = 0.02*abs(accLon);
            control.throttle = 0.0f;
            ls = crpc::VehicleLightState::LightState::Brake;
        }

        vehicle->SetLightState(ls);
        vehicle->ApplyControl(control);
    };

    thread camThread([&]()
    {
        while (playStatus)
        {
            unique_lock<mutex> lk(playCondVarMtx);
            playCondVar.wait(lk);
            // set the camera:
            auto spectator = world.GetSpectator();
            cg::Transform transform;
            Matrix4f rotM1; rotM1.setIdentity(); rotM1.block(0,0,3,3) = AngleAxisf( M_PI_2, Vector3f::UnitY()).toRotationMatrix();
            Matrix4f rotM2; rotM2.setIdentity(); rotM2.block(0,0,3,3) = AngleAxisf(-M_PI_2, Vector3f::UnitZ()).toRotationMatrix();
            Matrix4f mirror; mirror.setIdentity(); mirror(1,1) = -1;
            Matrix4f CAM = mirror*camTrf.inverse()*rotM2*rotM1;
            transform.location = cg::Location(CAM(0,3), CAM(1,3), CAM(2,3));
            float pitch = asin(CAM(2,0));
            float yaw = atan2(CAM(1,0), CAM(0,0));
            Matrix3f rotM = AngleAxisf(-pitch, Vector3f::UnitY())*AngleAxisf(-yaw, Vector3f::UnitZ())*CAM.block(0,0,3,3);
            float roll = -asin(rotM(2,1));
            transform.rotation = cg::Rotation(pitch*RAD2DEG, yaw*RAD2DEG, roll*RAD2DEG);
            spectator->SetTransform(transform);
        }
    });

    thread driveThread([&]()
    {
        while (playStatus)
        {
            unique_lock<mutex> lk(playCondVarMtx);
            playCondVar.wait(lk);
                
            auto * actor = &vehicles[0];
            auto scenario_vehicles = scenario.children();
            for (auto it = scenario_vehicles.begin(); it != scenario_vehicles.end(); ++it, ++actor)
            {
                if (!playStatus)
                    break;

                auto vehicle = static_cast<cc::Vehicle*>(actor->get());

                auto trf = vehicle->GetTransform();
                auto heading = (vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
                auto speed =  vehicle->GetVelocity().Length();

                Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(it->second);
                Waypath & waypath = *dynamic_cast<Waypath*>(scenario_vehicle.children().begin()->second);

                // Get the next waypoints in some distance away:
                Eigen::Vector3f peigen(trf.location.x, -trf.location.y, trf.location.z);
                float targetSpeed;
                Eigen::Vector3f targetDir;
                if (!waypath.getNext(peigen, targetDir, targetSpeed, speed, FPS))
                {
                    cc::Vehicle::Control control;
                    control.brake = 1.0f;
                    control.throttle = 0.0f;
                    auto ls = vehicle->GetLightState();
                    ls = crpc::VehicleLightState::LightState::Brake;
                    vehicle->SetLightState(ls);
                    vehicle->ApplyControl(control);
                    continue;
                }
                cg::Vector3D dir(targetDir.x(), -targetDir.y(), targetDir.z());
                auto arc = dir - heading; // actually this is a chord, but it is close to arc for small angles
                auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;

                float wdir = sign * arc.Length();
                // The stronger is the curvature the lower speed:
                float curvature = abs(sin(0.5f*wdir))/vehicle->GetBoundingBox().extent.x;
                float accLat = speed*speed*curvature;
                float accLon = (targetSpeed - speed);
                applyControl(accLat, accLon, vehicle, targetSpeed, 0.2*wdir*900/70); // 900/70 is typical ratio steer-wheels
                Actor * visuactor = dynamic_cast<Actor*>(scenario.children()[scenario_vehicle.getID()]);
                if (visuactor)
                {
                    visuactor->setTrf(trf.location.x, -trf.location.y, trf.location.z, 0, 0, -trf.rotation.yaw);
                    visuactor->setBbox(Vector3f(vehicle->GetBoundingBox().extent.x, vehicle->GetBoundingBox().extent.y, vehicle->GetBoundingBox().extent.z));
                }
            }
        }
    });

    while (playStatus)
    {
        try
        {
            std::chrono::time_point<std::chrono::system_clock> timenow = std::chrono::system_clock::now();
            if (1 == playStatus)
            {
                unique_lock<mutex> lk(playCondVarMtx);
                playCondVar.wait(lk);
            }
            // unfortunately we cannot Tick for Navigation only - in this case the Actors will live "on their onw"
            mw->update();
            playCondVar.notify_all();
            world.Tick(carla::time_duration(1s));

            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - timenow).count();
            if (realtime_playback)
                usleep(std::max(0.0, (1.0 * 1e6 / FPS - diff)));
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    for (auto v : vehicles) v->Destroy();
    camThread.join();
    driveThread.join();

    usleep(1e6);
}


