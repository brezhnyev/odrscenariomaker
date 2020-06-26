#include "treeview.h"
#include "treeitem.h"

#include <QtCore/QItemSelectionModel>

TreeView::TreeView(Scenario & scenario) : QTreeView(), m_scenario(scenario)
{
    m_treeModel = new TreeModel("", this);
    setModel(m_treeModel);
    connect(selectionModel(), &QItemSelectionModel::selectionChanged, 
    [this](const QItemSelection & sel, const QItemSelection & desel){
        auto sellist = selectionModel()->selection().indexes();
        if (sellist.empty()) return; // KB: IMHO should not happen. However happens due to slot_select (blockSignals do not help)
        int s = sellist.size();
        auto treeItem = static_cast<TreeItem*>(sellist[0].internalPointer());
        auto ti2 = static_cast<TreeItem*>(sellist[1].internalPointer());
        selectionModel()->selection();
        emit signal_select(treeItem->getID());
    });
}

void TreeView::slot_addWaypath(int id)
{
    m_treeModel->addWaypath(id);
}

void TreeView::slot_addWaypoint(int id)
{
    m_treeModel->addWaypoint(m_scenario.m_activeWaypath, id);
}

void TreeView::slot_delWaypath(int) {}
void TreeView::slot_delWaypoint(int) {}
void TreeView::slot_select(int id)
{
    selectionModel()->clear();
    blockSignals(true);
    selectionModel()->select(m_treeModel->getIndexById(id), QItemSelectionModel::Rows | QItemSelectionModel::Select);
    blockSignals(false);
}