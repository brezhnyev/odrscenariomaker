#pragma once

#include "treemodel.h"
#include "scenario.h"

#include <QtWidgets/QTreeView>

class TreeView : public QTreeView
{
    Q_OBJECT
public:
    TreeView(Scenario & scenario);
    void loadScenario();

public slots:
    void slot_addItem(int, std::string);
    void slot_addWaypoint(int);
    void slot_delItem(int);
    void slot_select(int);

signals:
    void signal_select(int id);

private:
    void addItem(int id, Selectable *);

private:
    TreeModel   * m_treeModel;
    Scenario & m_scenario;
    int m_scenarioID{0};
    int m_selectedItemID{-1};
};