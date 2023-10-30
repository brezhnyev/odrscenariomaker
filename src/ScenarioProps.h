#pragma once

#include "scenario.h"

#include <QtWidgets/QWidget>

class ScenarioProps : public QWidget
{
    Q_OBJECT
public:
    ScenarioProps(Scenario & actor);

signals:
    void signal_addVehicle(int);
    void signal_addWalker(int);
    void signal_addCamera(int);
    void signal_update(QString);
    void signal_clear();

protected:
    Scenario & m_scenario;
};