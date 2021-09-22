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
    void signal_delVehicle(int);
    void signal_updateOnScenarioLoad();

protected:
    Scenario & m_scenario;
};