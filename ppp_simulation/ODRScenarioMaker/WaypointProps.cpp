#include "WaypointProps.h"

#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>

#define MINMAXPOS 1000000

WaypointProps::WaypointProps(Waypoint & p) : m_waypoint(p)
{
    QVBoxLayout * l1 = new QVBoxLayout();
    QDoubleSpinBox * x = new QDoubleSpinBox(this);
    x->setRange(-MINMAXPOS, MINMAXPOS);
    x->setSingleStep(0.1);
    x->setValue(p.getPosition().x());
    QDoubleSpinBox * y = new QDoubleSpinBox(this);
    y->setRange(-MINMAXPOS, MINMAXPOS);
    y->setSingleStep(0.1);
    y->setValue(p.getPosition().y());
    QDoubleSpinBox * z = new QDoubleSpinBox(this);
    z->setRange(-MINMAXPOS, MINMAXPOS);
    z->setSingleStep(0.1);
    z->setValue(p.getPosition().z());

    l1->addWidget(new QLabel("X", this));
    l1->addWidget(x);
    l1->addWidget(new QLabel("Y", this));
    l1->addWidget(y);
    l1->addWidget(new QLabel("Z", this));
    l1->addWidget(z);

    QGroupBox * posInfo = new QGroupBox("Positions", this);
    posInfo->setLayout(l1);

    QDoubleSpinBox * speed = new QDoubleSpinBox(this);
    speed->setValue(p.getSpeed());
    QGroupBox * speedInfo = new QGroupBox("Speed", this);
    QHBoxLayout * l2 = new QHBoxLayout();
    l2->addWidget(speed);
    speedInfo->setLayout(l2);

    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Waypoint " + QString::number(p.getID()));
    lh->addWidget(idInfo);
    lh->addWidget(posInfo);
    lh->addWidget(speedInfo);

    lh->addStretch(1);

    QPushButton * delButton = new QPushButton(this);
    delButton->setText("Delete");
    connect(delButton, &QPushButton::pressed, [this]()
    {
        int id = m_waypoint.getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypoint: index not found!");
        }
        emit signal_delete(id);
        close();
    });
    lh->addWidget(delButton);

    connect(x, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [y,z,this](double val){ m_waypoint.setPosition(Eigen::Vector3f(val, y->value(), z->value())); emit signal_update(); });
    connect(y, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [x,z,this](double val){ m_waypoint.setPosition(Eigen::Vector3f(x->value(), val, z->value())); emit signal_update(); });
    connect(z, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [x,y,this](double val){ m_waypoint.setPosition(Eigen::Vector3f(x->value(), y->value(), val)); emit signal_update(); });
    connect(speed, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double val){ m_waypoint.setSpeed(val); emit signal_update(); });
    
    setLayout(lh);
}