#pragma once

#include "treemodel.h"

#include <QtWidgets/QTreeView>

class TreeView : public QTreeView
{
    Q_OBJECT
public:
    TreeView(int scenarioID);
    // make it template to decouple from any Selectable classes
    template<typename T>
    void loadScenario(T * el)
    {
        m_treeModel->delItem(m_scenarioID);
        addItem(-1, el);
        m_scenarioID = el->getID();
        m_selectedItemID = m_scenarioID;
    }

public slots:
    void slot_addItem(int, std::string, int parentID = -1);
    void slot_delItem(int);
    void slot_select(int);

signals:
    void signal_select(int id);

private:
    // In case the item has children add all its children (this is possible on loading scenario)
    // make it template to decouple from any Selectable classes
    template<typename T>
    void addItem(int id, T * el)
    {
        m_treeModel->addItem(id, el->getID(), el->getType());
        for (auto child : el->children())
            addItem(el->getID(), child.second);
    }
    TreeModel   * m_treeModel;
    int m_scenarioID{0};
    int m_selectedItemID{-1};
};