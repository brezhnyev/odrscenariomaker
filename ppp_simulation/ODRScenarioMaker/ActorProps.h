#pragma once

#include "Vehicle.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>


class ActorProps : public QWidget
{
    Q_OBJECT
public:
    ActorProps(Actor & actor);

signals:
    void signal_update();
    void signal_delete(int);
    void signal_addWaypath(int);
    void signal_addCamera(int);

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