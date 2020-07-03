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
    virtual std::string getType() const = 0;

    virtual int addChild(Selectable * child);
    virtual int delChild(int id);

    int getID() const { return m_id; }
    Selectable * getActiveChild(int depth)
    { 
        return (m_activeChild == -1 || m_children.empty()) ? nullptr: !depth ? m_children[m_activeChild] : m_children[m_activeChild]->getActiveChild(depth - 1);
    }
    std::map<int, Selectable*> children() { return m_children; }
    
protected:
    static int          s_ID;
    int                 m_id;
    bool                m_selected;

    int                         m_activeChild;
    std::map<int, Selectable*>  m_children;
};