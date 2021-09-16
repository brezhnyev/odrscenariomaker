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
#include "MainWindow.h"

#define DEG2RAD M_PI/180


namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace crpc = carla::rpc;

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

bool doStop;
extern MainWindow * mw;
static string delim_ = ",";
ofstream hlpout;
static int FPS = 10;

static map<string, string> carla2ppp
{
    {"vehicle.audi.a2", "car"},
    {"vehicle.tesla.model3", "car"},
    {"vehicle.bh.crossbike", "bicycle"},
    {"vehicle.bmw.grandtourer", "car"},
    {"vehicle.audi.etron", "car"},
    {"vehicle.seat.leon", "car"},
    {"vehicle.volkswagen.t2", "truck"},
    {"vehicle.kawasaki.ninja", "motorcycle"},
    {"vehicle.mustang.mustang", "car"},
    {"vehicle.tesla.cybertruck", "car"},
    {"vehicle.lincoln.mkz2017", "car"},
    {"vehicle.lincoln2020.mkz2020", "car"},
    {"vehicle.dodge_charger.police", "car"},
    {"vehicle.gazelle.omafiets", "bicycle"},
    {"vehicle.yamaha.yzf", "motorcycle"},
    {"vehicle.audi.tt", "car"},
    {"vehicle.jeep.wrangler_rubicon", "car"},
    {"vehicle.harley-davidson.low_rider", "motorcycle"},
    {"vehicle.chevrolet.impala", "car"},
    {"vehicle.nissan.patrol", "car"},
    {"vehicle.nissan.micra", "car"},
    {"vehicle.mercedesccc.mercedesccc", "car"},
    {"vehicle.bmw.isetta", "car"},
    {"vehicle.mini.cooperst", "car"},
    {"vehicle.chargercop2020.chargercop2020", "car"},
    {"vehicle.toyota.prius", "car"},
    {"vehicle.mercedes-benz.coupe", "car"},
    {"vehicle.diamondback.century", "bicycle"},
    {"vehicle.citroen.c3", "car"},
    {"vehicle.charger2020.charger2020", "car"},
    {"vehicle.carlamotors.carlacola", "car"}
};

string getHLPHeader()
{
    return "timestamp" + delim_ +
           "Obstacles_UniqueId" + delim_ +
           "Obstacles_Type" + delim_ +
           "Obstacles_RelativePosition_X" + delim_ +
           "Obstacles_RelativePosition_Y" + delim_ +
           "Obstacles_RelativePosition_Z" + delim_ +
           "Obstacles_RelativeVelocity_X" + delim_ +
           "Obstacles_RelativeVelocity_Y" + delim_ +
           "Obstacles_RelativeVelocity_Z" + delim_ +
           "Obstacles_RelativeSpeed" + delim_ +
           "Obstacles_Length" + delim_ +
           "Obstacles_Width" + delim_ +
           "Obstacles_Height" + delim_ +
           "Obstacles_RelativeRoll" + delim_ +
           "Obstacles_RelativePitch" + delim_ +
           "Obstacles_RelativeYaw" + delim_ +
           "Obstacles_TimeToCollision" + delim_ +
           "Obstacles_RelativeLane" + delim_ +
           "Obstacles_AbsoluteLane" + delim_ +
           "Ego_AbsolutePosition_X" + delim_ +
           "Ego_AbsolutePosition_Y" + delim_ +
           "Ego_AbsolutePosition_Z" + delim_ +
           "Ego_AbsoluteVelocity_X" + delim_ +
           "Ego_AbsoluteVelocity_Y" + delim_ +
           "Ego_AbsoluteVelocity_Z" + delim_ +
           "Ego_AbsoluteSpeed" + delim_ +
           "Ego_Roll" + delim_ +
           "Ego_Pitch" + delim_ +
           "Ego_Yaw" + delim_ +
           "Ego_YawRate" + delim_ +
           "Ego_AbsoluteLane" + delim_ +
           "Ego_Lattitude" + delim_ +
           "Ego_Longitude" + delim_ +
           "Ego_Atttitude" + delim_ +
           "Ego_Acceleration_X_Raw" + delim_ +
           "Ego_Acceleration_Y_Raw" + delim_ +
           "Ego_Acceleration_Z_Raw" + delim_ +
           "\n";
}

void printObject(uint64_t timestamp, ShrdPtrActor obstacle, ShrdPtrActor ego)
{
    hlpout << timestamp << delim_;
    if (obstacle)
    {
        hlpout << obstacle->GetId() << delim_;
        hlpout << carla2ppp[obstacle->GetDisplayId()] << delim_;
        Matrix4d om, em; // obstacle and ego matrix
        om.setIdentity();
        em.setIdentity();
        auto otrf = obstacle->GetTransform();
        auto etrf = ego->GetTransform();
        om.block(0,0,3,3) = AngleAxisd(otrf.rotation.roll*DEG2RAD, Vector3d::UnitX())*AngleAxisd(otrf.rotation.pitch*DEG2RAD, Vector3d::UnitY())*AngleAxisd(-otrf.rotation.yaw*DEG2RAD, Vector3d::UnitZ()).toRotationMatrix();
        om.block(0,3,3,1) = Vector3d(otrf.location.x, -otrf.location.y, otrf.location.z);
        em.block(0,0,3,3) = AngleAxisd(etrf.rotation.roll*DEG2RAD, Vector3d::UnitX())*AngleAxisd(etrf.rotation.pitch*DEG2RAD, Vector3d::UnitY())*AngleAxisd(-etrf.rotation.yaw*DEG2RAD, Vector3d::UnitZ()).toRotationMatrix();
        em.block(0,3,3,1) = Vector3d(etrf.location.x, -etrf.location.y, etrf.location.z);
        // relative transformation:
        Matrix4d trf = em.inverse()*om;
        Matrix3d rotM = trf.block(0,0,3,3);
        double yaw = atan2(rotM(1,0),rotM(0,0));
        double pitch = asin(rotM(2,0));
        rotM = rotM*AngleAxis<double>(-yaw, Vector3d::UnitZ())*AngleAxis<double>(-pitch, Vector3d::UnitY());
        double roll = asin(rotM(2,1));
        hlpout << trf(0,3) << delim_ << trf(1,3) << delim_ << trf(2,3) << delim_;
        // relative velocity:
        auto vel = obstacle->GetVelocity() - ego->GetVelocity(); // velocity relative to EGO in World CS
        Vector3d rvel = em.block(0,0,3,3).inverse()*Vector3d(vel.x, -vel.y, vel.z); // velocity relative to EGO in EGO CS
        hlpout << rvel.x() << delim_ << rvel.y() << delim_ << rvel.z() << delim_;
        hlpout << rvel.norm() << delim_; // speed
        hlpout << 2*obstacle->GetBoundingBox().extent.x << delim_ << 2*obstacle->GetBoundingBox().extent.y << delim_ << 2*obstacle->GetBoundingBox().extent.z << delim_;
        // roll pitch yaw:
        hlpout << roll << delim_ << pitch << delim_ << yaw << delim_;
        hlpout << delim_ << delim_ << delim_; // time-to-collision, relative-lane, absolute-lane
    }
    else for (int i = 0; i < 18; ++i) hlpout << delim_;
    auto trf = ego->GetTransform();
    hlpout << trf.location.x << delim_ << -trf.location.y << delim_ << trf.location.z << delim_;
    auto vel = ego->GetVelocity();
    hlpout << vel.x << delim_ << -vel.y << delim_ << vel.z << delim_;
    hlpout << vel.Length() << delim_; // speed
    hlpout << trf.rotation.roll*DEG2RAD << delim_ << trf.rotation.pitch*DEG2RAD << delim_ << -trf.rotation.yaw*DEG2RAD << delim_;
    for (int i = 0; i < 8; ++i) hlpout << delim_;
    hlpout << endl;

}

int play(Scenario & scenario)
{
    doStop = false;
    hlpout.open("hlp.csv");
    hlpout << getHLPHeader();

    auto client = cc::Client("127.0.0.1", 2000);
    client.SetTimeout(10s);

    cout << "Client API version : " << client.GetClientVersion() << '\n';
    cout << "Server API version : " << client.GetServerVersion() << '\n';

    auto world = client.LoadWorld("Town02");

    // Synchronous mode:
    auto defaultSettings = world.GetSettings();
    crpc::EpisodeSettings wsettings(true, false, 1.0 / FPS); // (synchrone, noRender, interval)
    world.ApplySettings(wsettings, carla::time_duration::seconds(10));
    world.SetWeather(crpc::WeatherParameters::ClearNoon);

    // Spawn Vehicles:
    auto scenario_vehicles = scenario.children();
    const int number_of_vehicles = scenario_vehicles.size();
    auto spawn_points = world.GetMap()->GetRecommendedSpawnPoints();
    vector<ShrdPtrActor> vehicles; vehicles.reserve(number_of_vehicles);

    cc::Vehicle::Control control;

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
    }
    world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

    // Set spectator trf by the first waypoint of the first vehicle:
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

    uint64_t frameID = 0;
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
                Actor * visuactor = dynamic_cast<Actor*>(scenario.children()[scenario_vehicle.getID()]);
                if (visuactor) visuactor->setTrf(trf.location.x, -trf.location.y, trf.location.z, -trf.rotation.yaw);

                if (vehicles.size() == 1) // only ego vehicle
                    printObject((double)frameID/FPS*1000000000ull, nullptr, vehicles[0]);
                else if (actor != &vehicles[0])
                    printObject((double)frameID/FPS*1000000000ull, *actor, vehicles[0]);
            }
            mw->update();
            world.Tick(carla::time_duration(1s));
            ++frameID;
        }
        catch(exception & e) { cout << "Ignoring exception: " << e.what() << endl; }
    }

    world.ApplySettings(defaultSettings, carla::time_duration::seconds(10));
    for (auto v : vehicles) v->Destroy();
    usleep(1e6);

    hlpout.close();
}


