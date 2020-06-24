#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTreeView>
#include "treemodel.h"

#include "Viewer.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget * parent = nullptr);

private:
    Viewer      *m_viewer;
    QTreeView   *m_treeView;
    TreeModel   *m_treeModel;
};