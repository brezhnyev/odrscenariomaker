#pragma once

#include "CanvasXODR.h"

#include <QGLViewer/qglviewer.h>
#include <sys/time.h>

#include <thread>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer(std::string xodrfile, float radius, float xodrResolution);
    ~Viewer() override;
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;
    void mousePressEvent(QMouseEvent * event) override;

private:
    CanvasXODR      m_canvas;
    timeval         mMClTime;
};
