#pragma once

#include "Vehicle.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>


class ActorProps : public QWidget
{
    Q_OBJECT
public:
    ActorProps(Actor & actor);

signals:
    void signal_addWaypath(int);
    void signal_delWaypath(int);

protected:
    Actor & m_actor;
    QLabel * m_idInfo;
};

class VehicleProps : public ActorProps
{
    Q_OBJECT
public:
    VehicleProps(Vehicle & actor);
};