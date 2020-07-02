#pragma once

class Selectable
{
public:
    Selectable();
    virtual ~Selectable() {}
    virtual Selectable * getChild(int id) = 0;
    virtual void draw() = 0;
    virtual void drawWithNames() = 0;
    virtual bool select(int) = 0;

    int getID() { return m_id; }
    
protected:
    static int          s_ID;
    int                 m_id;
    bool                m_selected;
};