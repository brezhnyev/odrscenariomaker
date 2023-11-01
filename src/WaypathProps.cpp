#include "WaypathProps.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>

WaypathProps::WaypathProps(Waypath & p, std::list<QMetaObject::Connection> & cons) : m_waypath(p)
{
    QVBoxLayout * lh = new QVBoxLayout();

    QLabel * info = new QLabel("Use Shift+Left Mouse to set points in 3D View");
    info->setWordWrap(true);
    lh->addWidget(info);
    QPushButton * delButton = new QPushButton("Delete", this);
    delButton->setStyleSheet("background-color: red");
    lh->addWidget(delButton);
    lh->addStretch(1);

    setLayout(lh);
    cons.push_back(connect(delButton, &QPushButton::clicked, [this]()
    {
        int id = m_waypath.getID();
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypoint: index not found!");
        }
        emit signal_delete(id);
    }));
}