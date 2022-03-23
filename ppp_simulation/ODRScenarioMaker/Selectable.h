#pragma once

#include <map>

/** Drawable is added as Basis for the Selectable.
 * The class is introduce to solve the problem of rendering the CatmullRom computed intermediate points of the Waypoints.
 * Drawable does not have ID and does not have children
 */
class Drawable
{
public:
    Drawable() {}
    virtual ~Drawable() {}
    // the current design assums that the Drawable does not have children
    virtual void draw() = 0;
};

/** Selectable is the base class for all objects that can be selected (with mouse) and listed in the tree.
 * Unlike Drawable this class does have the global unique ID and it is assumed that it also can have children.
 */
class Selectable : public Drawable
{
public:
    Selectable();
    virtual Selectable * findSelectable(int id);
    void draw() override;
    virtual void drawWithNames();
    virtual bool select(int);
    virtual std::string getType() const = 0;
    virtual std::string getName() const { return "unnamed"; }

    virtual int addChild(Selectable * child);
    virtual int delChild(int id);

    int getID() const { return m_id; }
    void setID(int id) { m_id = id; s_ID = std::max(s_ID, m_id); } // relevant when reading from file / deserializing
    Selectable * getActiveChild(int depth)
    { 
        return (m_activeChild == -1 || m_children.empty()) ? nullptr: !depth ? m_children[m_activeChild] : m_children[m_activeChild]->getActiveChild(depth - 1);
    }
    std::map<int, Selectable*> & children() { return m_children; }
    void clear() { clearRecursively(this); }
    
protected:
    static int                  s_ID;
    int                         m_id;
    bool                        m_selected;

    int                         m_activeChild;
    std::map<int, Selectable*>  m_children;

protected:
    virtual void drawGeometry() {};

private:
    void clearRecursively(Selectable * s);
};