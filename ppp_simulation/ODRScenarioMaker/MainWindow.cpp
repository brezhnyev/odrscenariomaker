#include "MainWindow.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QDockWidget>

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent)
{
    m_viewer = new Viewer();
    setCentralWidget(m_viewer);

    m_treeModel = new TreeModel("", this);
    m_treeView = new QTreeView();
    m_treeView->setModel(m_treeModel);
    QDockWidget *dock = new QDockWidget(tr("Paths"), this);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
    dock->setWidget(m_treeView);
}