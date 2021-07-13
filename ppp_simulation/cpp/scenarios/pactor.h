#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/Waypoint.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>

#include <string>

enum PEvent { COLLISION };

typedef carla::SharedPtr<carla::client::Actor> CShrpActor; // carla shared pointer to Actor

class PActor
{
public:
    PActor(carla::client::World & w, std::string name, carla::geom::Transform trf, float speed, std::vector<std::string> behaviour) 
    : m_world(w), m_name(name), m_trf(trf), m_speed(speed), m_behaviour(behaviour) {}
    virtual ~PActor()
    {
        m_actor->Destroy();
    }
    virtual void Tick() = 0;
    virtual void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) = 0;
protected:
    CShrpActor m_actor;
    carla::client::World & m_world;
    std::string m_name;
    carla::geom::Transform m_trf;
    float m_speed;
    std::vector<std::string> m_behaviour;
};

class PVehicle : public PActor
{
public:
    PVehicle(carla::client::World & world, std::string name, carla::geom::Transform trf, float speed, std::vector<std::string> behaviour);
    void Tick() override;
    void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) override {};
private:
    carla::client::Vehicle::Control m_control;
    std::map<int, std::vector<carla::SharedPtr<carla::client::Waypoint>>> m_paths;
    int m_pathID;
    float m_dist;
    bool m_turnLeft;
};

class PWalker : public PActor
{
public:
    PWalker(carla::client::World & world, std::string name, carla::geom::Transform trf, float speed, std::vector<std::string> behaviour);
    void Tick() override;
    void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) override {};
private:
    carla::client::Walker::Control m_control;
};