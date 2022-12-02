#pragma once

#include "Camera.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

class CameraProps : public QWidget
{
    Q_OBJECT
public:
    CameraProps(Camera & actor);

signals:
    void signal_delete(int);

protected:
    Camera & m_camera;
};
