#include "WaypathProps.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>

WaypathProps::WaypathProps(Waypath & p) : m_waypath(p)
{
    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Waypath " + QString::number(p.getID()));

    QPushButton * delLastPoint = new QPushButton(this);
    delLastPoint->setText("Delete last waypoint");

    lh->addWidget(idInfo);
    lh->addWidget(delLastPoint);

    setLayout(lh);
    // delChild(0) with dummy 0 parameter. The overriden delChild will pop the last waypoint from waypath
    connect(delLastPoint, &QPushButton::pressed, [this]()
    {
        int id = m_waypath.delChild(0);
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypoint: index not found!");
        }
        emit signal_delWaypoint(id);
    });

    QPushButton * updateWaypath = new QPushButton(this);
    updateWaypath->setText("Update Waypath");

    lh->addWidget(updateWaypath);
    connect(updateWaypath, &QPushButton::pressed, [this]()
    {
        m_waypath.updateSmoothPath();
        emit signal_updateSmoothPath();
    });
    lh->addStretch(1);
}