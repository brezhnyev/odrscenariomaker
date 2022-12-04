#include "Selectable.h"

#include <string>
#include <algorithm>

using namespace std;

int Selectable::s_ID = 0;

Drawable::~Drawable() {}

Selectable::~Selectable() {}

Selectable::Selectable(Selectable * parent) : m_selected(false), m_parent(parent)
{ 
    m_id = s_ID; ++s_ID;
}

void Selectable::draw() const
{
    for (auto && c : m_children) c.second->draw();
}

void Selectable::drawWithNames() const
{
    for (auto && c : m_children) c.second->drawWithNames();
}

int Selectable::addChild(Selectable * object)
{
    m_children[object->getID()] = object;
    return object->getID();
}

void Selectable::deleteSelectable(int id)
{
    Selectable * s = findSelectable(id);
    Selectable * p = s->m_parent;
    s->clear();
    p->m_children.erase(id);
}

void Selectable::deleteSelectable(Selectable * s)
{
    int id = s->getID();
    Selectable * p = s->m_parent;
    s->clear();
    s->m_children.erase(id);
}

void Selectable::select(int id)
{
    parse([id](Selectable * object)
    {
        if (object->getID() == id)
            object->m_selected = true;
        else
            object->m_selected = false;
    });
}

Selectable * Selectable::getActiveChild(int depth, int cDepth)
{
    if (cDepth >= depth && m_selected)
        return this;
    for (auto && child : m_children)
    {
        Selectable * sel = child.second->getActiveChild(depth, cDepth+1);
        if (sel)
        {
            if (cDepth == depth)
                return this;
            else
                return sel;
        }
    }
    return nullptr;
}

void Selectable::parse(function<void(Selectable *)> fun)
{
    for (auto && child : m_children)
    {
        child.second->parse(fun);
    }
    fun(this);
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

void Selectable::clear()
{
    parse([](Selectable * object)
    {
        object->children().clear();
        // exception: we cannot delete Scenario, its a stack varialbe, referenced in classes
        if (object->getType() != "Scenario")
            delete object;
    });
}

Selectable * Selectable::getSelected()
{
    Selectable * selected = nullptr;
    parse([&](Selectable * s)
    {
        if (s->m_selected)
            selected = s;
    });
    return selected;
}