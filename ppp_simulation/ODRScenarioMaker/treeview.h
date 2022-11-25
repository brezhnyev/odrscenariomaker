#pragma once

#include "treemodel.h"
#include "Selectable.h"

#include <QtWidgets/QTreeView>

class TreeView : public QTreeView
{
    Q_OBJECT
public:
    TreeView(int scenarioID);
    void loadScenario(Selectable * scenario);

public slots:
    void slot_addItem(int, std::string, int parentID = -1);
    void slot_delItem(int);
    void slot_select(int);

signals:
    void signal_select(int id);

private:
    void addItem(int, Selectable *);
    TreeModel   * m_treeModel;
    int m_scenarioID{0};
    int m_selectedItemID{-1};
};