#include "Selectable.h"

int Selectable::s_ID = 0;

Selectable::Selectable() : m_selected(false), m_activeChild(0)
{ 
    ++s_ID; m_id = s_ID;
}

void Selectable::draw()
{
    for (auto && c : m_children) c.second->draw();
}

void Selectable::drawWithNames()
{
    for (auto && c : m_children) c.second->drawWithNames();
}

int Selectable::addChild(Selectable * object)
{
    m_children[object->getID()] = object;
    m_activeChild = object->getID();
    return m_activeChild;
}

int Selectable::delChild(int id)
{
    if (m_children.empty()) return -1;

    m_children.erase(id);

    if (m_children.empty()) m_activeChild = -1;
    else m_activeChild = 0;

    return id;
}

bool Selectable::select(int id)
{
    // otherwise selet the waypoint only:
    m_activeChild = 0;
    m_selected = false;

    for (auto && child : m_children)
    {
        if (child.second->select(id))
        {
            m_activeChild = child.second->getID();
        }
    }

    // if path is selected then select all points on the path:
    if (id == m_id)
    {
        for (auto && child : m_children) child.second->select(child.second->getID());
        m_selected = true;
    }

    return m_selected || m_activeChild;
}

Selectable * Selectable::getChild(int id)
{
    Selectable * selection = nullptr;

    for (auto && child : m_children)
    {
        if (child.second->getID() == id)
        {
            return child.second;
        }
        selection = child.second->getChild(id); // empty function for point
        if (selection) return selection;
    }
    return selection;
}