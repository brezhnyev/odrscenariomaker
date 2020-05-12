#include <string>
#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/rpc/EpisodeSettings.h>

#include <thread>
#include <vector>

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

class PPPScene
{
public:
    PPPScene(std::string config);
    void start();
    void stop();

private:
    void setRGBCams(std::string config, std::vector<ShrdPtrActor> & cams, std::string blueprintName);
    void setLidar(std::string config);

private:
    ShrdPtrActor                            m_actor;
    std::vector<ShrdPtrActor>               m_RGBCams;
    std::vector<ShrdPtrActor>               m_RGBCamsSS; // semantic segmentation
    carla::client::World                   *m_world;
    carla::rpc::EpisodeSettings             m_defaultSettings;

    std::thread                            *m_thread;
    bool m_doRun;
};