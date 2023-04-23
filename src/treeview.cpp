#include "treeview.h"
#include "Selectable.h"

#include <QtCore/QItemSelectionModel>
#include <QtWidgets/QMessageBox>

#include <string>

TreeView::TreeView(TreeItem * root) : QTreeView()
{
    m_treeModel = new TreeModel(root, this);
    setModel(m_treeModel);
    connect(selectionModel(), &QItemSelectionModel::selectionChanged, 
    [this](const QItemSelection & sel, const QItemSelection & desel){
        auto sellist = sel.indexes();
        if (sellist.empty()) return; // KB: IMHO should not happen. However happens due to slot_select (blockSignals do not help)
        auto treeItem = static_cast<TreeItem*>(sellist[0].internalPointer());
        emit signal_select(treeItem->getID());
    });
}

void TreeView::slot_addItem(int id)
{
    m_treeModel->layoutChanged();
    emit signal_select(id);
}

void TreeView::slot_delItem(int id)
{
    selectionModel()->clearSelection();
    m_treeModel->layoutChanged();
}

void TreeView::slot_select(int id)
{
    setExpanded(m_treeModel->getIndexById(id), true);
    selectionModel()->clear();
    blockSignals(true);
    selectionModel()->select(m_treeModel->getIndexById(id), QItemSelectionModel::Rows | QItemSelectionModel::Select);
    blockSignals(false);
}