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
    void update();

private:
    QDoubleSpinBox *m_x;
    QDoubleSpinBox *m_y;
    QDoubleSpinBox *m_z;
    QDoubleSpinBox *m_speed;
    Waypoint & m_waypoint;
};