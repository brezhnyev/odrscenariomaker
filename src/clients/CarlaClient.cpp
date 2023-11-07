#include "Client.h"
#include "MainWindow.h"

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

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

#ifdef USE_CARLA

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

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

typedef carla::SharedPtr<cc::Actor> ShrdPtrActor;

void CarlaClient::play(Scenario & scenario)
{
    m_playStatus = PLAY; // 0 stop, 1 pause, 2 play
    m_camTrf.setIdentity();

    auto client = cc::Client("127.0.0.1", 2000);
    auto const TIMEOUT = 10s;
    client.SetTimeout(TIMEOUT);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    try
    {
        cout << "Trying to connect to server, waiting for " << TIMEOUT.count() << " seconds" << endl;
        cout << "Server API version : " << client.GetServerVersion() << '\n';
    }
    catch(...)
    {
        cerr << "Cannot connect to server." << endl;
        return;
    }

    if (!scenario.get_townName().empty())
    {
        try { client.LoadWorld(scenario.get_townName()); }
        catch(...) { cout << "Check Town name spelling! The town does not exist." << endl; return; }
    }

    auto world = scenario.get_townName().empty() ? client.GetWorld() : client.LoadWorld(scenario.get_townName());
 
    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(m_isSynchronous, false, 1.0 / m_FPS); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    auto weather = world.GetWeather();
    weather.fog_density = 0.0f; // higher saturation
    weather.mie_scattering_scale = 0.0f; // better sky contrast
    weather.sun_altitude_angle = 80.0f;
    weather.cloudiness = 10.0f;
    world.SetWeather(weather);

    vector<ShrdPtrActor> vehicles;
    vector<ShrdPtrActor> walkers;
    std::vector<ShrdPtrActor> cameras; // carla cameras
    std::vector<Camera*> scenario_cameras;

    auto addCam = [&](Camera * camera, ShrdPtrActor actor)
    {
        scenario_cameras.push_back(camera);
        auto blueprint_library = world.GetBlueprintLibrary();
        // Find a camera blueprint.
        auto camera_bp = const_cast<cc::BlueprintLibrary::value_type*>(blueprint_library->Find("sensor.camera.rgb"));

        camera_bp->SetAttribute("image_size_x", to_string(camera->get_width()));
        camera_bp->SetAttribute("image_size_y", to_string(camera->get_height()));
        camera_bp->SetAttribute("fov", to_string(camera->get_FOV()));

        auto camera_transform = cg::Transform{
            cg::Location{camera->get_pos().x(), -camera->get_pos().y(), camera->get_pos().z()},        // x, y, z.
            cg::Rotation{-camera->get_ori().y(), -camera->get_ori().z(), camera->get_ori().x()}}; // pitch, yaw, roll.

        cameras.push_back(world.SpawnActor(*camera_bp, camera_transform, actor.get()));
        camera->get_camWidget()->resize(camera->get_width(), camera->get_height());
        camera->get_camWidget()->show();

        // Register a callback to save images to disk.
        ((cc::Sensor*)cameras.back().get())->Listen([camera](auto data)
        {
            auto image = boost::static_pointer_cast<csd::Image>(data);
            QPixmap backBuffer = QPixmap::fromImage(QImage((unsigned char*)image->data(), image->GetWidth(), image->GetHeight(), QImage::Format_RGBX8888).rgbSwapped());
            camera->get_camWidget()->setPixmap(backBuffer.scaled(camera->get_camWidget()->size(), Qt::KeepAspectRatio));
            camera->get_camWidget()->update();
        });
    };

    for (auto && child : scenario.children())
    {
        Camera * scenario_camera = dynamic_cast<Camera*>(child);
        if (scenario_camera)
        {
            addCam(scenario_camera, nullptr);
        }
        else // Vehicle or Walker 
        {
            Actor * scenario_actor = dynamic_cast<Actor*>(child);
            if (scenario_actor)
            {
                auto blueprint = (*world.GetBlueprintLibrary()->Filter(scenario_actor->get_name()))[0];
                if (blueprint.ContainsAttribute("color"))
                {
                    // auto &attribute = blueprint.GetAttribute("color");
                    blueprint.SetAttribute("color", scenario_actor->colorToString()); // does not have effect in Carla 0.9.12 (bug?)
                }
                ShrdPtrActor actor = nullptr;
                // Initialize Actor with first position of the first Waypath:
                Waypath * waypath = scenario_actor->getFirstWaypath();
                if (waypath && !waypath->children().empty())
                {
                    waypath->updateSmoothPath();
                    Eigen::Vector3f dir = waypath->getStartingDirection();
                    Eigen::Vector3f pos = waypath->getStartingPosition();
                    auto yaw = (atan2(-dir.y(), dir.x()))*90/M_PI_2; // Since Carla is LEFT handed - flip Y
                    cg::Transform transform(cg::Location(pos.x(), -pos.y(), pos.z()+0.5), cg::Rotation(0,yaw,0)); // Since Carla is LEFT handed - flip Y
                    // Spawn the vehicle.
                    actor = world.TrySpawnActor(blueprint, transform);
                }
                if (actor)
                {
                    cout << "Spawned " << actor->GetDisplayId() << '\n';
                    auto vehicle = dynamic_cast<cc::Vehicle *>(actor.get());
                    if (vehicle)
                    {
                        vehicle->SetSimulatePhysics();
                        vehicles.push_back(actor);
                    }
                    if (dynamic_cast<cc::Walker *>(actor.get()))
                    {
                        walkers.push_back(actor);
                    }
                    for (auto && child : scenario_actor->children())
                    {
                        Camera * camera = dynamic_cast<Camera*>(child);
                        if (camera)
                        {
                            addCam(camera, actor);
                        }
                    }
                }
                else
                    cout << "Failed to spawn actor ------" << endl;
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
        while (PLAY == m_playStatus)
        {
            unique_lock<mutex> lk(m_playCondVarMtx);
            m_playCondVar.wait(lk);
            // set the camera:
            auto spectator = world.GetSpectator();
            cg::Transform transform;
            Matrix4f rotM1; rotM1.setIdentity(); rotM1.block(0,0,3,3) = AngleAxisf( M_PI_2, Vector3f::UnitY()).toRotationMatrix();
            Matrix4f rotM2; rotM2.setIdentity(); rotM2.block(0,0,3,3) = AngleAxisf(-M_PI_2, Vector3f::UnitZ()).toRotationMatrix();
            Matrix4f mirror; mirror.setIdentity(); mirror(1,1) = -1;
            Matrix4f CAM = mirror*m_camTrf.inverse()*rotM2*rotM1;
            transform.location = cg::Location(CAM(0,3), CAM(1,3), CAM(2,3));
            float pitch = asin(CAM(2,0));
            float yaw = atan2(CAM(1,0), CAM(0,0));
            Matrix3f rotM = AngleAxisf(-pitch, Vector3f::UnitY())*AngleAxisf(-yaw, Vector3f::UnitZ())*CAM.block(0,0,3,3);
            float roll = -asin(rotM(2,1));
            transform.rotation = cg::Rotation(pitch*RAD2DEG, yaw*RAD2DEG, roll*RAD2DEG);
            spectator->SetTransform(transform);
        }
    });

    while (m_playStatus != STOP)
    {
        std::chrono::time_point<std::chrono::system_clock> timenow = std::chrono::system_clock::now();
        if (PAUSE == m_playStatus)
        {
            unique_lock<mutex> lk(m_playCondVarMtx);
            m_playCondVar.wait(lk);
        }
        
        size_t carla_actor_c = 0;
        for (auto && scenario_actor : scenario.children())
        {
            if (scenario_actor->getType() != "Vehicle")
                continue;

            cc::Vehicle * carla_vehicle = dynamic_cast<cc::Vehicle*>(vehicles[carla_actor_c].get());

            auto trf = carla_vehicle->GetTransform();
            auto heading = (carla_vehicle->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
            auto speed =  carla_vehicle->GetVelocity().Length();

            Vehicle & scenario_vehicle = *dynamic_cast<Vehicle*>(scenario_actor);
            Waypath * waypath = scenario_vehicle.getFirstWaypath(); // cannot process multiple paths now
            if (!waypath) // can be camera
                continue;

            // Get the next waypoints in some distance away:
            Eigen::Vector3f peigen(trf.location.x, -trf.location.y, trf.location.z);
            float targetSpeed;
            Eigen::Vector3f targetDir;
            if (!waypath->getNext(peigen, targetDir, targetSpeed, speed, m_FPS))
            {
                cc::Vehicle::Control control;
                control.brake = 1.0f;
                control.throttle = 0.0f;
                auto ls = carla_vehicle->GetLightState();
                ls = crpc::VehicleLightState::LightState::Brake;
                carla_vehicle->SetLightState(ls);
                carla_vehicle->ApplyControl(control);
                ++carla_actor_c;
                if (carla_actor_c >= vehicles.size())
                    break;
                continue;
            }
            cg::Vector3D dir(targetDir.x(), -targetDir.y(), targetDir.z());
            auto sign = (dir.x * heading.y - dir.y * heading.x) > 0 ? -1 : 1;
            float wheelsAngle = acos(dir.x*heading.x + dir.y*heading.y + dir.z*heading.z);
            // assuming max weelAngle is 70 degrees # for each car is individual, need to be looked up
            wheelsAngle = min<float>(1.0f, wheelsAngle/1.22173);
            float accLon = (targetSpeed - speed);
            applyControl(0, accLon, carla_vehicle, targetSpeed, sign*wheelsAngle);
            scenario_vehicle.setTrf(trf.location.x, -trf.location.y, trf.location.z, 0, 0, -trf.rotation.yaw);
            scenario_vehicle.set_bbox(Vector3f(carla_vehicle->GetBoundingBox().extent.x, carla_vehicle->GetBoundingBox().extent.y, carla_vehicle->GetBoundingBox().extent.z));

            ++carla_actor_c;
            if (carla_actor_c >= vehicles.size())
                break;
        }
        carla_actor_c = 0;
        for (auto && scenario_actor : scenario.children())
        {
            if (scenario_actor->getType() != "Walker")
                continue;

            cc::Walker * carla_walker = dynamic_cast<cc::Walker*>(walkers[carla_actor_c].get());

            auto trf = carla_walker->GetTransform();
            auto heading = (carla_walker->GetTransform().GetForwardVector()).MakeUnitVector(); // it may already be unit vector
            auto speed =  carla_walker->GetVelocity().Length();

            Walker & scenario_walker = *dynamic_cast<Walker*>(scenario_actor); 
            Waypath * waypath = scenario_walker.getFirstWaypath(); // cannot process multiple paths now
            if (!waypath)
                continue;

            // Get the next waypoints in some distance away:
            Eigen::Vector3f peigen(trf.location.x, -trf.location.y, trf.location.z);
            float targetSpeed;
            Eigen::Vector3f targetDir;
            cc::Walker::Control wc = carla_walker->GetWalkerControl();
            if (!waypath->getNext(peigen, targetDir, targetSpeed, speed, m_FPS))
            {
                wc.speed = 0;
            }
            else
            {
                wc.speed = targetSpeed;
                wc.direction = cg::Vector3D(targetDir[0], -targetDir[1], targetDir[2]);
            }
            carla_walker->ApplyControl(wc);
            scenario_walker.setTrf(trf.location.x, -trf.location.y, trf.location.z - carla_walker->GetBoundingBox().extent.z, 0, 0, -trf.rotation.yaw);
            scenario_walker.set_bbox(Vector3f(carla_walker->GetBoundingBox().extent.x, carla_walker->GetBoundingBox().extent.y, carla_walker->GetBoundingBox().extent.z));
            ++carla_actor_c;
            if (carla_actor_c >= walkers.size())
                break;
        }
        m_window->update();
        if (m_isSynchronous )
        {
            try { world.Tick(carla::time_duration(1s)); }
            catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
            m_playCondVar.notify_all();
            auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - timenow).count();
            if (m_realtimePlayback)
                usleep(std::max(0.0, (1.0 * 1e6 / m_FPS - diff)));
        }
        else
        {
            world.WaitForTick(carla::time_duration(1s));
            m_playCondVar.notify_all();
        }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    // close the qt widgets
    for (auto c : scenario_cameras) c->get_camWidget()->close();
    for (auto c : cameras) c->Destroy();
    for (auto v : vehicles) v->Destroy();
    for (auto w : walkers) w->Destroy();
    camThread.join();

    for (auto && child : scenario.children())
    {
        if (child->getType() == "Vehicle" || child->getType() == "Walker")
        {
            for (auto && wp : child->children())
            {
                if (wp->getType() == "Waypath")
                {
                    Waypath * waypath = dynamic_cast<Waypath*>(wp);
                    waypath->updateSmoothPath(); // resets the Actor to initial position
                }
            }
        }
        m_window->update();
    }

    usleep(1e6);
}
#endif


void CarlaClient::playDummy(Scenario & scenario)
{
    m_playStatus = PLAY; // 0 stop, 1 pause, 2 play
    m_camTrf.setIdentity();

    while (m_playStatus != STOP)
    {
        std::chrono::time_point<std::chrono::system_clock> timenow = std::chrono::system_clock::now();
        if (PAUSE == m_playStatus)
        {
            unique_lock<mutex> lk(m_playCondVarMtx);
            m_playCondVar.wait(lk);
        }
        
        for (auto && child : scenario.children())
        {
            if (child->getType() != "Vehicle" && child->getType() != "Walker")
                continue;

            Actor * actor = dynamic_cast<Actor*>(child);

            Waypath * waypath = actor->getFirstWaypath(); // cannot process multiple paths now
            if (!waypath) // can be camera
                continue;

            // Get the next waypoints in some distance away:
            Eigen::Vector3f peigen = actor->getPos();
            float targetSpeed;
            Eigen::Vector3f targetDir;
            if (!waypath->getNext(peigen, targetDir, targetSpeed, 1, m_FPS))
            {
                continue;
            }
            peigen += targetDir * targetSpeed*(1.0f/m_FPS);
            float yaw = atan2(targetDir[1], targetDir[0]);
            actor->setTrf(peigen, Vector3f(0, 0, yaw*RAD2DEG));
        }

        m_window->update();

        m_playCondVar.notify_all();
        auto diff = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - timenow).count();
        usleep(std::max(0.0, (1.0 * 1e6 / m_FPS - diff)));
    }

    for (auto && child : scenario.children())
    {
        if (child->getType() == "Vehicle" || child->getType() == "Walker")
        {
            for (auto && wp : child->children())
            {
                if (wp->getType() == "Waypath")
                {
                    Waypath * waypath = dynamic_cast<Waypath*>(wp);
                    waypath->updateSmoothPath(); // resets the Actor to initial position
                }
            }
        }
        m_window->update();
    }

    usleep(1e6);
}