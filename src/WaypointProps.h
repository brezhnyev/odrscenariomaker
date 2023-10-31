#pragma once

#include "Waypoint.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QDoubleSpinBox>

class LDoubleSpinBox;

class WaypointProps : public QWidget
{
    Q_OBJECT
public:
    WaypointProps(Waypoint & p, std::list<QMetaObject::Connection> &);
    void update(float x, float y, float z);

signals:
    void signal_update();
    void signal_delete(int);


private:
    Waypoint & m_waypoint;
    LDoubleSpinBox * x, *y, *z;
};