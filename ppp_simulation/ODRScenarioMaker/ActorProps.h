#pragma once

#include "Vehicle.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>


class ActorProps : public QWidget
{
    Q_OBJECT
public:
    ActorProps(Actor & actor);
    virtual ~ActorProps() {}

signals:
    void signal_addWaypath(int);
    void signal_delWaypath(int);
    void signal_update();

protected:
    Actor & m_actor;
};

class VehicleProps : public ActorProps
{
    Q_OBJECT
public:
    VehicleProps(Vehicle & actor);
    QPushButton * m_colorPicker; // apparently must be member, otherwise cannot properly capture in labda -> crash

private:
    Vehicle & m_vehicle;
};