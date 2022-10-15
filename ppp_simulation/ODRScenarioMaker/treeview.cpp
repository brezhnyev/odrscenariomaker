#include "treeview.h"
#include "treeitem.h"

#include <QtCore/QItemSelectionModel>
#include <QtWidgets/QMessageBox>

#include <string>

TreeView::TreeView(Scenario & scenario) : QTreeView(), m_scenario(scenario)
{
    m_treeModel = new TreeModel("", this);
    setModel(m_treeModel);
    connect(selectionModel(), &QItemSelectionModel::selectionChanged, 
    [this](const QItemSelection & sel, const QItemSelection & desel){
        auto sellist = sel.indexes();
        if (sellist.empty()) return; // KB: IMHO should not happen. However happens due to slot_select (blockSignals do not help)
        auto treeItem = static_cast<TreeItem*>(sellist[0].internalPointer());
        //auto ti2 = static_cast<TreeItem*>(sellist[1].internalPointer()); // not used
        emit signal_select(treeItem->getID());
    });
    m_treeModel->addItem(-1, m_scenario.getID(), "Scenario");
    m_scenarioID = scenario.getID();
}

void TreeView::addItem(int id, Selectable * item)
{
    m_treeModel->addItem(id, item->getID(), item->getType());
    // In case the item has children add all its children (this is possible on loading scenario)
    for (auto child : item->children())
        addItem(item->getID(), child.second);
}

void TreeView::loadScenario()
{
    m_treeModel->delItem(m_scenarioID);
    addItem(-1, &m_scenario);
    m_scenarioID = m_scenario.getID();
}

void TreeView::slot_addItem(int id, std::string name)
{
    m_treeModel->addItem(m_selectedItemID, id, name);
    emit signal_select(id);
}

// slot to add waypoint is separately implemented from slot_addItem
void TreeView::slot_addWaypoint(int id)
{
    m_treeModel->addItem(m_scenario.getActiveWaypathID(), id, "Waypoint");
    emit signal_select(id);
}

void TreeView::slot_delItem(int id)
{
    m_treeModel->delItem(id);
    selectionModel()->clearSelection();
}

void TreeView::slot_select(int id)
{
    m_selectedItemID = id;
    setExpanded(m_treeModel->getIndexById(m_selectedItemID), true);
    selectionModel()->clear();
    blockSignals(true);
    selectionModel()->select(m_treeModel->getIndexById(id), QItemSelectionModel::Rows | QItemSelectionModel::Select);
    blockSignals(false);
}