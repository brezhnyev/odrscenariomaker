#include <string>
#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/rpc/EpisodeSettings.h>

#include <thread>

class PPPScene
{
public:
    PPPScene(std::string config);
    void start();
    void stop();

private:
    void setSensors(std::string config);

private:
    carla::SharedPtr<carla::client::Actor>  m_actor;
    carla::SharedPtr<carla::client::Actor>  m_cam_actor;
    carla::client::World                   *m_world;
    carla::rpc::EpisodeSettings             m_defaultSettings;
    std::thread                            *m_thread;
    bool m_doRun;
};