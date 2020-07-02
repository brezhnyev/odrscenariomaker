#include "WaypathProps.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>


WaypathProps::WaypathProps(Waypath & p) : m_waypath(p)
{
    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Waypath " + QString::number(p.getID()));

    QPushButton * delLastPoint = new QPushButton(this);
    delLastPoint->setText("Delete last waypoint");

    lh->addWidget(idInfo);
    lh->addWidget(delLastPoint);
    lh->addStretch(1);

    setLayout(lh);
    // delChild(0) with dummy 0 parameter. The overriden delChild will pop the last waypoint from waypath
    connect(delLastPoint, &QPushButton::pressed, [this](){ int id = m_waypath.delChild(0); if (id != -1) emit signal_delWaypoint(id); });
}