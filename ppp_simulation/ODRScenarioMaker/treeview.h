#pragma once

#include "treemodel.h"
#include "scenario.h"

#include <QtWidgets/QTreeView>

class TreeView : public QTreeView
{
    Q_OBJECT
public:
    TreeView(Scenario & scenario);

public slots:
    void slot_addVehicle(int);
    void slot_addWalker(int);
    void slot_addWaypath(int);
    void slot_addWaypoint(int);
    void slot_delItem(int);
    void slot_select(int);

signals:
    void signal_select(int id);

private:
    TreeModel   * m_treeModel;
    Scenario & m_scenario;
};