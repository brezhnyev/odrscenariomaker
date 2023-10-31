#include "Selectable.h"

#include <string>
#include <algorithm>
#include <assert.h>

using namespace std;

int Selectable::s_ID = 0;
stack<string> Selectable::s_undo;
stack<string> Selectable::s_redo;
bool Selectable::s_allowUndo = true;

Selectable::~Selectable() {}

Selectable::Selectable(Selectable * parent) : m_selected(false), m_parent(parent), m_magicNumber(1176543210987654320LL)
{
    m_id = s_ID; ++s_ID;
    if (s_allowUndo)
        makeUndoSnapshot(parent);
    if (m_parent)
        m_parent->m_children[m_id] = this;
}

void Selectable::draw() const
{
    for (auto && c : m_children) c.second->draw();
}

void Selectable::drawWithNames() const
{
    for (auto && c : m_children) c.second->drawWithNames();
}

void Selectable::to_yaml(YAML::Node & parent)
{
    for (auto && child : m_children) child.second->to_yaml(parent);
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
    if (s_allowUndo)
        makeUndoSnapshot(this);
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

int Selectable::row() const
{
    if (!m_parent)
        return 0;
    int r = 0;
    for (auto & c : m_parent->children())
    {
        if (c.second == this)
            return r;
        ++r;
    }
    assert(false);
    return 0;
}

std::string Selectable::data(int index) const
{
    assert(index < 1);

    if (m_parent)
    {
        if (index == 0)
            return getType();
        // if (index == 1)
        //     return std::to_string(m_id);
    }
    else
    {
        if (index == 0)
            return "Type";
        // if (index == 1)
        //     return "Id";
    }
    return "";
}

void Selectable::makeUndoSnapshot(Selectable * parent)
{
    while (parent && parent->getType() != "Scenario") parent = parent->getParent();
    if (parent) // can be either nullptr or Scenario
    {
        s_redo = stack<string>();
        YAML::Node root;
        parent->to_yaml(root);
        stringstream ss; ss << root;
        s_undo.push(ss.str());
    }
}