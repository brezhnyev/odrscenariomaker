#include "Selectable.h"

#include <string>

using namespace std;

int Selectable::s_ID = 0;

Selectable::Selectable() : m_selected(false), m_activeChild(-1)
{ 
    m_id = s_ID; ++s_ID;
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
    m_activeChild = -1;

    return id;
}

bool Selectable::select(int id)
{
    m_activeChild = -1;
    m_selected = false;

    // if parent is selected select all children:
    if (id == m_id)
    {
        for (auto && child : m_children) child.second->select(child.second->getID());
        m_selected = true;
    }
    else for (auto && child : m_children)
    {
        if (child.second->select(id)) // effects in either select or deselect the children
        {
            m_activeChild = child.second->getID();
        }
    }
    return m_selected || m_activeChild != -1;
}

Selectable * Selectable::findSelectable(int id)
{
    if (id == m_id) return this;

    Selectable * selection = nullptr;

    for (auto && child : m_children)
    {
        if (selection = child.second->findSelectable(id)) break;
    }
    return selection;
}

void Selectable::clearRecursively(Selectable * s)
{
    for (auto && c : s->children())
    {
        if (c.second->children().empty()) delete c.second;
        else clearRecursively(c.second);
    }
    s->children().clear();
}