#include "CameraProps.h"

#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>

#include <eigen3/Eigen/Eigen>

#include <sstream>
#include <iostream>

#define MINMAXPOS 1000000

CameraProps::CameraProps(Camera & camera) : m_camera(camera)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();

    QDoubleSpinBox * x = new QDoubleSpinBox(this);
    x->setRange(-MINMAXPOS, MINMAXPOS);
    x->setSingleStep(0.01);
    x->setValue(camera.getPos().x());
    QDoubleSpinBox * y = new QDoubleSpinBox(this);
    y->setRange(-MINMAXPOS, MINMAXPOS);
    y->setSingleStep(0.01);
    y->setValue(camera.getPos().y());
    QDoubleSpinBox * z = new QDoubleSpinBox(this);
    z->setRange(-MINMAXPOS, MINMAXPOS);
    z->setSingleStep(0.01);
    z->setValue(camera.getPos().z());

    QVBoxLayout * l1 = new QVBoxLayout();
    l1->addWidget(new QLabel("X", this));
    l1->addWidget(x);
    l1->addWidget(new QLabel("Y", this));
    l1->addWidget(y);
    l1->addWidget(new QLabel("Z", this));
    l1->addWidget(z);

    QGroupBox * posInfo = new QGroupBox(this);
    posInfo->setTitle("Position");
    posInfo->setLayout(l1);
    mainLayout->addWidget(posInfo);

    connect(x, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [y,z,this](double val){ m_camera.setPos(Eigen::Vector3f(val, y->value(), z->value())); emit update(); });
    connect(y, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [x,z,this](double val){ m_camera.setPos(Eigen::Vector3f(x->value(), val, z->value())); emit update(); });
    connect(z, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [x,y,this](double val){ m_camera.setPos(Eigen::Vector3f(x->value(), y->value(), val)); emit update(); });

    QVBoxLayout * l2 = new QVBoxLayout();
    QDoubleSpinBox * roll = new QDoubleSpinBox(this);
    roll->setRange(-MINMAXPOS, MINMAXPOS);
    roll->setSingleStep(0.01);
    roll->setValue(camera.getOri().x());
    QDoubleSpinBox * pitch = new QDoubleSpinBox(this);
    pitch->setRange(-MINMAXPOS, MINMAXPOS);
    pitch->setSingleStep(0.01);
    pitch->setValue(camera.getOri().y());
    QDoubleSpinBox * yaw = new QDoubleSpinBox(this);
    yaw->setRange(-MINMAXPOS, MINMAXPOS);
    yaw->setSingleStep(0.01);
    yaw->setValue(camera.getOri().z());

    l2->addWidget(new QLabel("Roll", this));
    l2->addWidget(roll);
    l2->addWidget(new QLabel("Pitch", this));
    l2->addWidget(pitch);
    l2->addWidget(new QLabel("Yaw", this));
    l2->addWidget(yaw);

    QGroupBox * oriInfo = new QGroupBox(this);
    oriInfo->setTitle("Orientation");
    oriInfo->setLayout(l2);
    mainLayout->addWidget(oriInfo);

    connect(roll,  static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [pitch,yaw,this ](double val){ m_camera.setOri(Eigen::Vector3f(val, pitch->value(), yaw->value()));  emit update(); });
    connect(pitch, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [roll,yaw, this ](double val){ m_camera.setOri(Eigen::Vector3f(roll->value(), val,  yaw->value()));  emit update(); });
    connect(yaw,   static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [roll,pitch,this](double val){ m_camera.setOri(Eigen::Vector3f(roll->value(), pitch->value(), val)); emit update(); });

    QVBoxLayout * l3 = new QVBoxLayout();
    QDoubleSpinBox * FOV = new QDoubleSpinBox(this);
    FOV->setRange(-180, 180);
    FOV->setSingleStep(0.01);
    FOV->setValue(camera.getFOV());
    l3->addWidget(FOV);
    QGroupBox * intrinsics = new QGroupBox(this);
    intrinsics->setTitle("Cam props");
    intrinsics->setLayout(l3);
    mainLayout->addWidget(intrinsics);

    connect(FOV, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [this](double val){ m_camera.setFOV(val); emit update(); });

    QPushButton * delButton = new QPushButton("Delete", this);
    mainLayout->addStretch(1);
    mainLayout->addWidget(delButton);

    setLayout(mainLayout);
}