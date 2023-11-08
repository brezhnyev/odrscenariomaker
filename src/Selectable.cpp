#include "Selectable.h"
#include "globals.h"

#include <string>
#include <algorithm>
#include <assert.h>

using namespace std;

uint32_t Selectable::s_ID = 0;
stack<string> Selectable::s_undo;
stack<string> Selectable::s_redo;
bool Selectable::s_allowUndo = true;

Selectable::~Selectable() {}

Selectable::Selectable(Selectable * parent) : m_selected(false), m_parent(parent), m_magicNumber(MAGICNUMBER)
{
    m_id = s_ID; ++s_ID;
    if (s_allowUndo)
        makeUndoSnapshot(parent);
    if (m_parent)
        m_parent->children().push_back(this);
}

void Selectable::draw() const
{
    for (auto && c : m_children) c->draw();
}

void Selectable::drawWithNames() const
{
    for (auto && c : m_children) c->drawWithNames();
}

void Selectable::to_yaml(YAML::Node & parent)
{
    for (auto && child : m_children) child->to_yaml(parent);
}

void Selectable::deleteSelectable(int id)
{
    Selectable * s = findSelectable(id);
    Selectable * p = s->m_parent;
    s->clear();
    auto it = find_if(p->m_children.begin(), p->m_children.end(), [id](const Selectable * o){ return o->getID() == id; });
    p->m_children.erase(it);
}

void Selectable::deleteThis()
{
    clear();
    auto it = find_if(m_parent->m_children.begin(), m_parent->m_children.end(), [this](const Selectable * o){ return o == this; });
    m_parent->m_children.erase(it);
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
        Selectable * sel = child->getActiveChild(depth, cDepth+1);
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
        child->parse(fun);
    }
    fun(this);
}

Selectable * Selectable::findSelectable(int id)
{
    if (id == m_id) return this;
    Selectable * selection = nullptr;
    for (auto && child : m_children)
    {
        if (selection = child->findSelectable(id)) break;
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
        // this logics can be changed in future (ex. we can open multiple scenarios)
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
        if (c == this)
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