#pragma once

#include <QGLViewer/qglviewer.h>
#include <eigen3/Eigen/Eigen>

#include <mutex>
class Viewer : public QGLViewer
{
    Q_OBJECT

public:
    Viewer(
        std::vector<std::vector<Eigen::Vector2f>> && baseLines_,
        std::vector<Eigen::Vector2f> && baseLinesZ_,
        std::vector<std::vector<Eigen::Vector3f>> && centerlines,
        std::vector<std::vector<Eigen::Vector3f>>&& boundaries);
    void updateMovingObjects(std::vector<Eigen::Matrix4f> && v);
    
signals:
    void ClosingWindow();

protected :
    virtual void draw();
    virtual void init();
    void keyPressEvent(QKeyEvent *) override;

private:
    void drawSolidBox(const Eigen::Vector3f &, int id);
    void drawWireBox(const Eigen::Vector3f &, int id);
    void drawInfo();
    
private:
    std::vector<std::vector<Eigen::Vector2f>> baseLines_;
    std::vector<Eigen::Vector2f> baseLinesZ_;
    std::vector<std::vector<Eigen::Vector3f>> centerlines_;
    std::vector<std::vector<Eigen::Vector3f>> boundaries_;
    std::vector<Eigen::Matrix4f> actors_;
    std::mutex mtx_;
    std::vector<Eigen::Vector3f> colors_;
    bool enableLegend_;
};
