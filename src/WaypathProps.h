#pragma once

#include "Waypath.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QDoubleSpinBox>

class WaypathProps : public QWidget
{
    Q_OBJECT
public:
    WaypathProps(Waypath & p, std::list<QMetaObject::Connection> &);

signals:
    void signal_delete(int);

private:
    Waypath & m_waypath;
};