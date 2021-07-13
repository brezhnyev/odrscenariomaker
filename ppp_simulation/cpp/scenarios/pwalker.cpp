#include "pactor.h"

#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>

#include <iostream>

using namespace carla::geom;
using namespace carla::client;
using namespace std;

PWalker::PWalker(carla::client::World & world, std::string name, carla::geom::Transform trf, float speed, std::vector<string> behaviour) 
: PActor(world, name, trf, speed, behaviour)
{
    auto blueprints = world.GetBlueprintLibrary()->Filter(name);
    if (!blueprints->empty())
    {
        auto blueprint = (*blueprints)[0];
        if (blueprint.ContainsAttribute("is_invincible")) blueprint.SetAttribute("is_invincible", "false");
        m_actor = m_world.TrySpawnActor(blueprint, trf);

        // Store the walker and its controller
        cout << "Spawned " << m_actor->GetDisplayId() << '\n';
        m_world.Tick(carla::time_duration(1s)); // to set the transform of the vehicle

        m_control =  static_cast<Walker*>(m_actor.get())->GetWalkerControl();
        //m_world.SetPedestriansCrossFactor(0.0f);
    }
}

void PWalker::Tick()
{
    m_control.speed = m_speed;
    float yaw = m_trf.rotation.yaw*M_PI/180;
    m_control.direction = Vector3D(cos(yaw), sin(yaw),0);
    static_cast<Walker*>(m_actor.get())->ApplyControl(m_control);
}