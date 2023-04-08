#pragma once

#include "Vehicle.h"
#include "Walker.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>


class ActorProps : public QWidget
{
    Q_OBJECT
public:
    virtual ~ActorProps() = 0;
    ActorProps(Actor & actor);
    QPushButton * m_colorPicker; // apparently must be member, otherwise cannot properly capture in labda -> crash

signals:
    void signal_update();
    void signal_delete(int);
    void signal_addWaypath(int);
    void signal_addCamera(int);

protected:
    void addTypes(const QStringList & ls);

protected:
    QPushButton * m_delButton{nullptr};
    Actor & m_actor;
};

class VehicleProps : public ActorProps
{
    Q_OBJECT
public:
    VehicleProps(Vehicle & actor);

private:
    Vehicle & m_vehicle;
};


class WalkerProps : public ActorProps
{
    Q_OBJECT
public:
    WalkerProps(Walker & actor);

private:
    Walker & m_walker;
};