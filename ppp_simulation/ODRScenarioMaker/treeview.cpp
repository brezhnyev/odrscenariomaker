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
        //auto ti2 = static_cast<TreeItem*>(sellist[1].internalPointer()); // not used
        emit signal_select(treeItem->getID());
    });
    m_treeModel->addItem(-1, m_scenario.getID(), "Scenario");
    m_scenarioID = scenario.getID();
}

void TreeView::addItem(int id, Selectable * item)
{
    m_treeModel->addItem(id, item->getID(), item->getType());
    for (auto child : item->children())
        addItem(item->getID(), child.second);
}

void TreeView::loadScenario()
{
    m_treeModel->delItem(m_scenarioID);
    addItem(-1, &m_scenario);
    m_scenarioID = m_scenario.getID();
}

void TreeView::slot_addVehicle(int id)
{
    m_treeModel->addItem(m_scenario.getID(), id, "Vehicle");
}

void TreeView::slot_addWalker(int id)
{
    m_treeModel->addItem(m_scenario.getID(), id, "Walker");
}

void TreeView::slot_addWaypath(int id)
{
    m_treeModel->addItem(m_scenario.getActiveActorID(), id, "Waypath");
}

void TreeView::slot_addWaypoint(int id)
{
    m_treeModel->addItem(m_scenario.getActiveWaypathID(), id, "Waypoint");
}

void TreeView::slot_delItem(int id)
{
    m_treeModel->delItem(id);
}

void TreeView::slot_select(int id)
{
    selectionModel()->clear();
    blockSignals(true);
    selectionModel()->select(m_treeModel->getIndexById(id), QItemSelectionModel::Rows | QItemSelectionModel::Select);
    blockSignals(false);
}