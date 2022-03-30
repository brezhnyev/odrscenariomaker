#pragma once

#include "Waypoint.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QDoubleSpinBox>

class WaypointProps : public QWidget
{
    Q_OBJECT
public:
    WaypointProps(Waypoint & p);

signals:
    void signal_update();
    void signal_delete(int);

private:
    Waypoint & m_waypoint;
};