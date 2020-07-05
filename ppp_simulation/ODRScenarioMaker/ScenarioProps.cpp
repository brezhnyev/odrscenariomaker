#include "ScenarioProps.h"
#include "Vehicle.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QLabel>

#define MINMAXPOS 1000000

ScenarioProps::ScenarioProps(Scenario & scenario) : m_scenario(scenario)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    QLabel * m_idInfo = new QLabel(this);
    m_idInfo->setText(QString(scenario.getType().c_str()) + " ID: " + QString::number(scenario.getID()));

    QPushButton * addVehicle = new QPushButton(this);
    addVehicle->setText("Add vehicle");

    mainLayout->addWidget(m_idInfo);
    mainLayout->addWidget(addVehicle);

    QVBoxLayout * bl1 = new QVBoxLayout();
    QComboBox * delCombo = new QComboBox(this);
    for (auto && child : m_scenario.children()) delCombo->addItem(QString::number(child.second->getID()));
    QPushButton * delButton = new QPushButton(this);
    delButton->setText("Delete");
    bl1->addWidget(delButton);
    bl1->addWidget(delCombo);
    QGroupBox * delGroup = new QGroupBox(this);
    delGroup->setTitle("Delete vehicle");
    delGroup->setLayout(bl1);
    mainLayout->addWidget(delGroup);

    setLayout(mainLayout);

    connect(addVehicle, &QPushButton::pressed, [this, delCombo]()
    { 
        int id = m_scenario.addChild(new Vehicle());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error adding Element", "Failed to add Vehicle: index not found!");
            return;
        }
        emit signal_addVehicle(id);
        delCombo->clear();
        for (auto && child : m_scenario.children()) delCombo->addItem(QString::number(child.second->getID()));
    });
    connect(delButton, &QPushButton::pressed, [this, delCombo]()
    { 
        int id = m_scenario.delChild(delCombo->currentText().toInt());
        if (id == -1)
        {
            QMessageBox::warning(this, "Error deleting Element", "Failed to delete Vehicle: index not found!");
            return;
        }
        emit signal_delVehicle(id);
        delCombo->clear();
        for (auto && child : m_scenario.children()) delCombo->addItem(QString::number(child.second->getID()));
    });
}