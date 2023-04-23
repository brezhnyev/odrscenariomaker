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
    virtual void draw() const = 0;
    virtual void drawWithNames() const = 0;
};

/** Selectable is the base class for all objects that can be selected (with mouse) and is displayed in the scene tree.
 * Unlike Drawable this class does have the global unique ID and it is assumed that it also can have children.
 */
class Selectable : public Drawable
{
public:
    Selectable(Selectable * parent);
    virtual ~Selectable() = 0;
    void draw() const override;
    void drawWithNames() const override;
    virtual void select(int);
    virtual std::string getType() const = 0;

    void deleteSelectable(int);
    void deleteSelectable(Selectable *);

    Selectable * findSelectable(int id);
    int getID() const                       { return m_id; }
    void setID(int id)                      { m_id = id; s_ID = std::max(s_ID, m_id); } // relevant when reading from file / deserializing
    std::map<int, Selectable*> & children() { return m_children; }
    void clear();
    void parse(std::function<void(Selectable*)> fun);
    Selectable * getParent() const          { return m_parent; }
    void setParent(Selectable * parent)     { m_parent = parent; } // used only once in copy c-tor of Scenario (loading scenario)
    Selectable * getSelected();

    int row() const;
    int columnCount() const { return 2; }
    std::string data(int index) const;

protected:
    static int                  s_ID;
    int                         m_id;
    bool                        m_selected;
    std::map<int, Selectable*>  m_children;
    Selectable *                m_parent{nullptr};

protected:
    virtual void drawGeometry() const {};
    Selectable * getActiveChild(int depth, int cDepth = 0);
};

class Root : public Selectable
{
public:
    Root() : Selectable(nullptr) {}
    std::string getType() const override { return "Root"; }
};

typedef Selectable TreeItem;