#pragma once

#include "CanvasXODR.h"

#include <QGLViewer/qglviewer.h>
#include <QGLViewer/vec.h>
#include <sys/time.h>
#include "FolderReader.h"

#include <thread>

class Viewer : public QGLViewer
{
    Q_OBJECT
public:
    Viewer(std::string xodrfile, float xodrResolution);
    ~Viewer() override;
    void draw() override;
    void drawWithNames() override;
    void init() override;
    void postSelection(const QPoint &point) override;
    void mousePressEvent(QMouseEvent * event) override;
    void mouseMoveEvent(QMouseEvent *) override;
    void keyPressEvent(QKeyEvent * event) override;

private:
    FolderReader<std::set<std::string, LexCompare>>  mFr;
    float           mXodrRes{0.1f};
    CanvasXODR   *  m_canvas;
    std::string     mInfo;
    qglviewer::Vec  mSelPoint;
    std::string     mCurrentFile;
    qglviewer::Vec  mRibbonStart, mRibbonCurr, mRibbonEnd;
};
