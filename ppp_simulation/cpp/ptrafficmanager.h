#include "pactor.h"

class PTrafficManager
{
public:
    ~PTrafficManager() { for (auto && a : m_actors) delete a; }
    void AddActor(PActor * actor)
    {
        m_actors.push_back(actor);
    }
    void Setup();
    void Tick()
    {
        for (auto && a : m_actors) a->Tick();
    }
    std::vector<PActor*> m_actors;
};