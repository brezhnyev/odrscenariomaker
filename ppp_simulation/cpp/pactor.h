#include <carla/client/Actor.h>
#include <carla/geom/Transform.h>
#include <carla/client/World.h>
#include <carla/client/Vehicle.h>
#include <carla/client/Walker.h>
#include <carla/client/Waypoint.h>

enum PEvent { COLLISION };

typedef carla::SharedPtr<carla::client::Actor> CShrpActor; // carla shared pointer to Actor

class PActor
{
public:
    PActor(carla::client::World & w) : m_world(w) {}
    virtual ~PActor() {}
    virtual void Tick() = 0;
    virtual void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) = 0;
protected:
    CShrpActor m_actor;
    carla::client::World & m_world;
};

class PVehicle : public PActor
{
public:
    PVehicle(carla::client::World & world, carla::geom::Transform trf);
    void Tick() override;
    void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) override {};
private:
    carla::client::Vehicle::Control m_control;
    float m_speed;
    std::map<int, std::vector<carla::SharedPtr<carla::client::Waypoint>>> m_paths;
    int m_pathID;
    float m_dist;
    bool m_turnLeft;
};

class PWalker : public PActor
{
public:
    PWalker(carla::client::World & world, carla::geom::Transform trf);
    void Tick() override;
    void OnEvent(PEvent, std::vector<std::shared_ptr<PActor>>) override {};
private:
    carla::client::Walker::Control m_control;
};