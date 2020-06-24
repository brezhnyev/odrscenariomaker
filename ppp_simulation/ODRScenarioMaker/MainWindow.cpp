#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent)
{
    m_viewer = new Viewer(m_scenario);
    setCentralWidget(m_viewer);

    m_treeView = new TreeView(m_scenario);
    QDockWidget *dock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
    dock->setWidget(m_treeView);

    connect(m_viewer, SIGNAL(signal_addWaypath(int)), m_treeView, SLOT(slot_addWaypath(int)));
    connect(m_viewer, SIGNAL(signal_addWaypoint(int)), m_treeView, SLOT(slot_addWaypoint(int)));
    connect(m_viewer, SIGNAL(signal_delWaypath(int)), m_treeView, SLOT(slot_delWaypath(int)));
    connect(m_viewer, SIGNAL(signal_delWaypoint(int)), m_treeView, SLOT(slot_delWaypoint(int)));
    connect(m_viewer, SIGNAL(signal_setActiveWaypath(int)), m_treeView, SLOT(slot_setActiveWaypath(int)));
    connect(m_viewer, SIGNAL(signal_setlectWaypoint(int)), m_treeView, SLOT(slot_setActiveWaypoint(int)));
}