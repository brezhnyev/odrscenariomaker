#pragma once

#include <QGLViewer/qglviewer.h>
#include <eigen3/Eigen/Eigen>


class Viewer : public QGLViewer
{
    Q_OBJECT

public:

    void addData(std::vector<Eigen::Vector3f> & v);

signals:
    void ClosingWindow();

protected :
    virtual void draw();
    virtual void init();
    
private:
    std::vector<std::vector<Eigen::Vector3f>> data;
};
