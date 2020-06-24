#include "treeview.h"
#include "treemodel.h"

#include "Viewer.h"

#include <QtWidgets/QMainWindow>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget * parent = nullptr);

private:
    Viewer      *m_viewer;
    TreeView    *m_treeView;
    Scenario     m_scenario;
};