#pragma once

#include <map>
#include <functional>
#include "yaml-cpp/yaml.h"

/** Selectable is the base class for all objects that can be selected (with mouse) and is displayed in the scene tree.
 * The class has unique ID, children, can be selected from scene tree and by mouse click.
 */
class Selectable
{
public:
    Selectable(Selectable * parent);
    virtual ~Selectable() = 0;
    virtual void draw() const;
    virtual void drawWithNames() const;
    virtual void select(int);
    virtual std::string getType() const = 0;
    virtual void to_yaml(YAML::Node & parent);
    virtual void from_yaml(const YAML::Node & parent) {};

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