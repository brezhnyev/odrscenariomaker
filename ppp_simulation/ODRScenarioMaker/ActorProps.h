#pragma once

#include "Actor.h"

#include <QtWidgets/QWidget>

class ActorProps : public QWidget
{
    Q_OBJECT
public:
    ActorProps(Actor & actor);

signals:
    void signal_addWaypath(int);

private:
    Actor & m_actor;
};