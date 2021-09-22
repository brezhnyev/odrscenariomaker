#pragma once

#include "Waypath.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QDoubleSpinBox>

class WaypathProps : public QWidget
{
    Q_OBJECT
public:
    WaypathProps(Waypath & p);

signals:
    void signal_delWaypoint(int);
    void signal_updateSmoothPath();

private:
    Waypath & m_waypath;
};