#include <carla/client/Actor.h>
#include <carla/client/Sensor.h>
#include <carla/client/World.h>
#include <carla/rpc/EpisodeSettings.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <string>
#include <thread>
#include <vector>

typedef carla::SharedPtr<carla::client::Actor> ShrdPtrActor;

class PPPScene
{
public:
    PPPScene(std::string config);
    void start();
    void stop(bool abort = false);
    void doRecording(bool record);
    bool isInitialized() { return m_isInitialized; }

private:
    void setRGBCams(std::vector<ShrdPtrActor> & cams, std::string blueprintName, std::string outname);
    void setDepthCams(std::vector<ShrdPtrActor> & cams, std::string blueprintName, std::string outname);
    void setWeather();
    void spawnVehicles();
    void spawnWalkers();
    void SaveImageToDisk(const carla::sensor::data::Image &image, int index, std::string type);
    void destroyIfAlive(ShrdPtrActor actor)
    {
        if (actor.get()->IsAlive()) actor.get()->Destroy();
    }

private:
    ShrdPtrActor                            m_actor;
    std::vector<ShrdPtrActor>               m_RGBCams;
    std::vector<ShrdPtrActor>               m_RGBCamsSS; // semantic segmentation
    std::vector<ShrdPtrActor>               m_DepthCams;
    std::vector<ShrdPtrActor>               m_DepthCamsSS; // semantic segmentation
    carla::SharedPtr<carla::client::World>  m_world;
    carla::rpc::EpisodeSettings             m_defaultSettings;
    int                                     m_timeout;

    std::vector<ShrdPtrActor>               m_vehicles;
    std::vector<ShrdPtrActor>               m_walkers;
    std::vector<ShrdPtrActor>               m_wControllers;

    std::thread                            *m_thread;
    bool                                    m_doRun;
    bool                                    m_isInitialized;
    std::string                             m_confname;
};