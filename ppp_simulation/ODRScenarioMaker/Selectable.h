#pragma once

#include <map>

class Selectable
{
public:
    Selectable();
    virtual ~Selectable() {}
    virtual Selectable * getChild(int id);
    virtual void draw();
    virtual void drawWithNames();
    virtual bool select(int);

    virtual int addChild(Selectable * child);
    virtual int delChild(int id);

    int getID() { return m_id; }
    Selectable * getActiveChild() { return m_activeChild == -1 ? nullptr: m_children[m_activeChild]; }
    std::map<int, Selectable*> children() { return m_children; }
    
protected:
    static int          s_ID;
    int                 m_id;
    bool                m_selected;

    int                         m_activeChild;
    std::map<int, Selectable*>  m_children;
};