#include "ActorProps.h"
#include "Waypath.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>

#define MINMAXPOS 1000000

ActorProps::ActorProps(Actor & actor) : m_actor(actor)
{
    QVBoxLayout * lh = new QVBoxLayout();
    QLabel * idInfo = new QLabel(this);
    idInfo->setText("Actor " + QString::number(actor.getID()));

    QPushButton * addWaypath = new QPushButton(this);
    addWaypath->setText("Add waypath");

    lh->addWidget(idInfo);
    lh->addWidget(addWaypath);

    QHBoxLayout * bl1 = new QHBoxLayout();
    QComboBox * combo = new QComboBox(this);
    for (auto && child : m_actor.children()) combo->addItem(QString::number(child.second->getID()));
    QPushButton * delButton = new QPushButton(this);
    delButton->setText("Delete");
    bl1->addWidget(combo);
    bl1->addWidget(delButton);
    QGroupBox * delGroup = new QGroupBox(this);
    delGroup->setTitle("Delete path");
    delGroup->setLayout(bl1);
    lh->addWidget(delGroup);

    lh->addStretch(1);

    setLayout(lh);

    connect(addWaypath, &QPushButton::pressed, [this, combo]()
    { 
        int id = m_actor.addChild(new Waypath());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Waypath: index not found!");
            return;
        }
        emit signal_addWaypath(id);
        combo->clear();
        for (auto && child : m_actor.children()) combo->addItem(QString::number(child.second->getID()));
    });
    connect(delButton, &QPushButton::pressed, [this, combo]()
    { 
        int id = m_actor.delChild(combo->currentText().toInt());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Waypath: index not found!");
            return;
        }
        emit signal_delWaypath(id);
        combo->clear();
        for (auto && child : m_actor.children()) combo->addItem(QString::number(child.second->getID()));
    });
}