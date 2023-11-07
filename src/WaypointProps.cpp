#include "WaypointProps.h"
#include "LDoubleSpinBox.h"

#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>

#define MINMAXPOS 1000000

WaypointProps::WaypointProps(Waypoint & p, std::list<QMetaObject::Connection> & cons) : m_waypoint(p)
{
    QVBoxLayout * l1 = new QVBoxLayout();
    x = new LDoubleSpinBox(this, p.get_pos().x(), -MINMAXPOS, MINMAXPOS, 0.1, "X");
    y = new LDoubleSpinBox(this, p.get_pos().y(), -MINMAXPOS, MINMAXPOS, 0.1, "Y");
    z = new LDoubleSpinBox(this, p.get_pos().z(), -MINMAXPOS, MINMAXPOS, 0.1, "Z");
    float val = p.get_speed();
    LDoubleSpinBox * speed = new LDoubleSpinBox(this, p.get_speed(), -1000, 1000, 0.1, "Speed");
    l1->addWidget(x);
    l1->addWidget(y);
    l1->addWidget(z);
    l1->addWidget(speed);

    QGroupBox * posInfo = new QGroupBox("Positions", this);
    posInfo->setLayout(l1);

    QVBoxLayout * lh = new QVBoxLayout();
    lh->addWidget(posInfo);

    QPushButton * delButton = new QPushButton(this);
    delButton->setText("Delete");
    cons.push_back(connect(delButton, &QPushButton::clicked, [this]()
    {
        int id = m_waypoint.getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypoint: index not found!");
        }
        emit signal_delete(id);
    }));
    delButton->setStyleSheet("background-color: red");
    lh->addWidget(delButton);
    lh->addStretch(1);

    cons.push_back(connect(x, &LDoubleSpinBox::valueChanged, [this](double val){ m_waypoint.set_pos(Eigen::Vector3f(val, y->value(), z->value())); emit signal_update(); }));
    cons.push_back(connect(y, &LDoubleSpinBox::valueChanged, [this](double val){ m_waypoint.set_pos(Eigen::Vector3f(x->value(), val, z->value())); emit signal_update(); }));
    cons.push_back(connect(z, &LDoubleSpinBox::valueChanged, [this](double val){ m_waypoint.set_pos(Eigen::Vector3f(x->value(), y->value(), val)); emit signal_update(); }));
    cons.push_back(connect(speed, &LDoubleSpinBox::valueChanged, [this](double val){ m_waypoint.set_speed(val); emit signal_update(); }));
    
    setLayout(lh);
}

void WaypointProps::update(float valx, float valy, float valz)
{
    x->setValue(valx);
    y->setValue(valy);
    z->setValue(valz);
}