#pragma once

#include <map>
#include <functional>

/** Drawable is added as Basis for the Selectable.
 * The class is introduced to solve the problem of rendering the CatmullRom computed intermediate points of the Waypoints.
 * Drawable does not have ID and does not have children and therefore is not displayed in the scene tree
 */
class Drawable
{
public:
    Drawable() {}
    virtual ~Drawable() = 0;
    // the current design assumes that the Drawable does not have children
    virtual void draw() = 0;
    virtual void drawWithNames() = 0;
};

/** Selectable is the base class for all objects that can be selected (with mouse) and is displayed in the scene tree.
 * Unlike Drawable this class does have the global unique ID and it is assumed that it also can have children.
 */
class Selectable : public Drawable
{
public:
    Selectable();
    virtual ~Selectable() = 0;
    void draw() override;
    void drawWithNames() override;
    virtual void select(int);
    virtual std::string getType() const = 0;

    int addChild(Selectable * child);
    int delChild(int id);

    Selectable * findSelectable(int id);
    int getID() const { return m_id; }
    void setID(int id) { m_id = id; s_ID = std::max(s_ID, m_id); } // relevant when reading from file / deserializing
    std::map<int, Selectable*> & children() { return m_children; }
    void clear(){ clear(this); }
    void parse(std::function<void(Selectable*)> fun);
    
protected:
    static int                  s_ID;
    int                         m_id;
    bool                        m_selected;
    std::map<int, Selectable*>  m_children;

protected:
    virtual void drawGeometry() {};
    Selectable * getActiveChild(int depth, int cDepth = 0);

private:
    void clear(Selectable * s);
};