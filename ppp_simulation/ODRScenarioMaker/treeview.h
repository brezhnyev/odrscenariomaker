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
    void slot_addWaypath(int);
    void slot_addWaypoint(int);
    void slot_delWaypath(int);
    void slot_delWaypoint(int);
    void slot_setActiveWaypath(int);
    void slot_setlectWaypoint(int);

private:
    TreeModel   * m_treeModel;
    Scenario    & m_scenario;
};