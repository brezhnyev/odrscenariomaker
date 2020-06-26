#include "WaypointProps.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>

#define MINMAXPOS 1000000

WaypointProps::WaypointProps(Waypoint & p) : m_waypoint(p)
{
    QVBoxLayout * l1 = new QVBoxLayout();
    m_x = new QDoubleSpinBox(this);
    m_x->setRange(-MINMAXPOS, MINMAXPOS);
    m_x->setSingleStep(0.1);
    m_x->setValue(p.getPosition().x());
    m_y = new QDoubleSpinBox(this);
    m_y->setRange(-MINMAXPOS, MINMAXPOS);
    m_y->setSingleStep(0.1);
    m_y->setValue(p.getPosition().y());
    m_z = new QDoubleSpinBox(this);
    m_z->setRange(-MINMAXPOS, MINMAXPOS);
    m_z->setSingleStep(0.1);
    m_z->setValue(p.getPosition().z());

    l1->addWidget(new QLabel("X"));
    l1->addWidget(m_x);
    l1->addWidget(new QLabel("Y"));
    l1->addWidget(m_y);
    l1->addWidget(new QLabel("Z"));
    l1->addWidget(m_z);

    QGroupBox * posInfo = new QGroupBox(this);
    posInfo->setTitle("Positions");
    posInfo->setLayout(l1);

    m_speed = new QDoubleSpinBox(this);
    QGroupBox * speedInfo = new QGroupBox(this);
    speedInfo->setTitle("Speed");
    QHBoxLayout * l2 = new QHBoxLayout();
    l2->addWidget(m_speed);
    speedInfo->setLayout(l2);

    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Waypoint " + QString::number(p.getID()));
    lh->addWidget(idInfo);
    lh->addWidget(posInfo);
    lh->addWidget(speedInfo);
    this->setLayout(lh);

    connect(m_x, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double val){ m_waypoint.setPosition(Eigen::Vector3f(val, m_y->value(), m_z->value())); emit update(); });
    connect(m_y, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double val){ m_waypoint.setPosition(Eigen::Vector3f(m_x->value(), val, m_z->value())); emit update(); });
    connect(m_z, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double val){ m_waypoint.setPosition(Eigen::Vector3f(m_x->value(), m_y->value(), val)); emit update(); });
}