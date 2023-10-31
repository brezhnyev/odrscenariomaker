#pragma once

#include <map>
#include <stack>
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
    std::map<int, Selectable*> & children() { return m_children; }
    void clear();
    void parse(std::function<void(Selectable*)> fun);
    Selectable * getParent() const          { return m_parent; }
    Selectable * getSelected();


    int row() const;
    int columnCount() const { return 1; } // previously used 2 to diplay id, see data() function
    std::string data(int index) const;
    const uint64_t m_magicNumber;
    
protected:
    static int                  s_ID;
    int                         m_id;
    bool                        m_selected;
    std::map<int, Selectable*>  m_children;
    Selectable *                m_parent{nullptr};

    static std::stack<std::string> s_undo;
    static std::stack<std::string> s_redo;
    static bool s_allowUndo;

protected:
    virtual void drawGeometry() const {};
    Selectable * getActiveChild(int depth, int cDepth = 0);
    void makeUndoSnapshot(Selectable *);
};

class Root : public Selectable
{
public:
    Root() : Selectable(nullptr) {}
    std::string getType() const override { return "Root"; }
};

typedef Selectable TreeItem;