#pragma once

#include "Camera.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

class LDoubleSpinBox;

class CameraProps : public QWidget
{
    Q_OBJECT
public:
    CameraProps(Camera & actor);
    void update(float x, float y, float z);

signals:
    void signal_update();
    void signal_delete(int);

protected:
    Camera & m_camera;
    LDoubleSpinBox * x;
    LDoubleSpinBox * y;
    LDoubleSpinBox * z;
};
