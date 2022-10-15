#include "Selectable.h"

#include <string>

using namespace std;

int Selectable::s_ID = 0;

Drawable::~Drawable() {}

Selectable::~Selectable() {}

Selectable::Selectable() : m_selected(false)
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
    // Possible race condition in Qt?
    // For this case check if the id was previously added
    if (m_children.find(object->getID()) != m_children.end())
        return object->getID();
    m_children[object->getID()] = object;
    return object->getID();
}

int Selectable::delChild(int id)
{
    // KB: for some reason delItem may be called 2 times!! Race condition in Qt?
    // For this case check if the id was previously deleted
    if (m_children.find(id) == m_children.end())
        return id;
    if (m_children.empty()) return -1;
    m_children.erase(id);
    return id;
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
    fun(this); // post-order traversal
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

void Selectable::clear(Selectable * s)
{
    for (auto && c : s->children())
    {
        if (c.second->children().empty()) delete c.second;
        else clear(c.second);
    }
    s->children().clear();
}