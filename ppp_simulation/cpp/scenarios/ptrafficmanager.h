#include "pactor.h"

class PTrafficManager
{
public:
    PTrafficManager(carla::client::World & world, std::string confname);
    ~PTrafficManager() { for (auto && a : m_actors) delete a; }
    void Tick()
    {
        for (auto && a : m_actors) a->Tick();
    }
private:
    std::vector<PActor*> m_actors;
};