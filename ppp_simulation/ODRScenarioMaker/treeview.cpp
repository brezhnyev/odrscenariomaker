#include "treeview.h"
#include "treeitem.h"

#include <QtCore/QItemSelectionModel>
#include <QtWidgets/QMessageBox>

#include <string>

TreeView::TreeView(int scenarioID) : QTreeView()
{
    m_treeModel = new TreeModel("", this);
    setModel(m_treeModel);
    connect(selectionModel(), &QItemSelectionModel::selectionChanged, 
    [this](const QItemSelection & sel, const QItemSelection & desel){
        auto sellist = sel.indexes();
        if (sellist.empty()) return; // KB: IMHO should not happen. However happens due to slot_select (blockSignals do not help)
        auto treeItem = static_cast<TreeItem*>(sellist[0].internalPointer());
        emit signal_select(treeItem->getID());
    });
    m_treeModel->addItem(-1, scenarioID, "Scenario");
    m_scenarioID = scenarioID;
}

void TreeView::loadScenario(Selectable * scenario)
{
    m_treeModel->delItem(m_scenarioID);
    addItem(-1, scenario);
    m_scenarioID = scenario->getID();
}

// In case the item has children add all its children (this is possible on loading scenario)
void TreeView::addItem(int id, Selectable * item)
{
    m_treeModel->addItem(id, item->getID(), item->getType());
    for (auto child : item->children())
        addItem(item->getID(), child.second);
}

void TreeView::slot_addItem(int id, std::string name, int parentID)
{
    m_treeModel->addItem(parentID == -1 ? m_selectedItemID : parentID, id, name);
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