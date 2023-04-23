#pragma once

#include "treemodel.h"

#include <QtWidgets/QTreeView>

class TreeView : public QTreeView
{
    Q_OBJECT
public:
    TreeView(TreeItem *);

public slots:
    void slot_addItem(int);
    void slot_delItem(int);
    void slot_select(int);

signals:
    void signal_select(int id);

private:
    TreeModel   * m_treeModel;
};