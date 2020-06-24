#include "treeview.h"

TreeView::TreeView(Scenario & scenario) : QTreeView(), m_scenario(scenario)
{
    m_treeModel = new TreeModel("", this);
    setModel(m_treeModel);
}

void TreeView::slot_addWaypath(int index)
{
    m_treeModel->addWaypath(index);
}

void TreeView::slot_addWaypoint(int index)
{
    m_treeModel->addWaypoint(m_scenario.m_activeWaypath, index);
}

void TreeView::slot_delWaypath(int) {}
void TreeView::slot_delWaypoint(int) {}
void TreeView::slot_setActiveWaypath(int) {}
void TreeView::slot_setlectWaypoint(int) {}