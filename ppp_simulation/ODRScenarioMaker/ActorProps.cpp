#include "ActorProps.h"
#include "Waypath.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

#define MINMAXPOS 1000000

ActorProps::ActorProps(Actor & actor) : m_actor(actor)
{
    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Actor " + QString::number(actor.getID()));

    QPushButton * addWaypath = new QPushButton(this);
    addWaypath->setText("Add waypath");

    lh->addWidget(idInfo);
    lh->addWidget(addWaypath);
    lh->addStretch(1);

    setLayout(lh);

    connect(addWaypath, &QPushButton::pressed, [this](){ int id = m_actor.addChild(new Waypath()); emit signal_addWaypath(id); });
}