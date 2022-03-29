#include "ScenarioProps.h"
#include "Vehicle.h"
#include "Serializer.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QLineEdit>

#include <fstream>
#include <string>
#include <sstream>

using namespace std;

#define MINMAXPOS 1000000


ScenarioProps::ScenarioProps(Scenario & scenario) : m_scenario(scenario)
{
    QVBoxLayout * mainLayout = new QVBoxLayout();
    QLabel * m_idInfo = new QLabel(this);
    m_idInfo->setText(QString(scenario.getType().c_str()) + " ID: " + QString::number(scenario.getID()));
    mainLayout->addWidget(m_idInfo);

    // addd rosbag file
    QGroupBox * rosGroup = new QGroupBox(this);
    QVBoxLayout * rosLayout = new QVBoxLayout();
    rosGroup->setLayout(rosLayout);

    rosLayout->addWidget(new QLabel("Rosbag file:", rosGroup));
    QLineEdit * rosBagfile = new QLineEdit(rosGroup);
    rosBagfile->setText(m_scenario.getRosbagFile().c_str());
    connect(rosBagfile, &QLineEdit::textChanged, [&](const QString & text){ m_scenario.setRosbagFile(text.toStdString()); });
    rosLayout->addWidget(rosBagfile);

    rosLayout->addWidget(new QLabel("Rosbag topic:", rosGroup));
    QLineEdit * rosTopic = new QLineEdit(rosGroup);
    rosTopic->setText(m_scenario.getRosbagTopic().c_str());
    connect(rosTopic, &QLineEdit::textChanged, [&](const QString & text){ m_scenario.setRosbagTopic(text.toStdString()); });
    rosLayout->addWidget(rosTopic);

    rosLayout->addWidget(new QLabel("Rosbag playback offset:"));
    QLineEdit * rosTimeOffset = new QLineEdit(rosGroup);
    rosTimeOffset->setText(QString::number(m_scenario.getRosbagOffset()));
    connect(rosTimeOffset, &QLineEdit::textChanged, [&](const QString & text){ m_scenario.setRosbagOffset(text.toFloat()); });
    rosLayout->addWidget(rosTimeOffset);
    mainLayout->addWidget(rosGroup);


    QPushButton * addVehicle = new QPushButton(this);
    addVehicle->setText("Add vehicle");
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

    QPushButton * loadScenario = new QPushButton();
    loadScenario->setText("Load Scenario");
    mainLayout->addWidget(loadScenario);
    connect(loadScenario, &QPushButton::pressed, [this, rosBagfile, rosTopic, rosTimeOffset](){
        QString name = QFileDialog::getOpenFileName(this, tr("Open Scenario"), "/home", tr("Scenarios (*.yaml)"));
        if (name.isEmpty()) return;
        
        ifstream ifs(name.toStdString());
        m_scenario.clear();
        stringstream ss; ss << ifs.rdbuf();
        m_scenario = Serializer::deserialize_yaml(ss.str());
        emit signal_updateOnScenarioLoad();
        rosBagfile->setText(m_scenario.getRosbagFile().c_str());
        rosTopic->setText(m_scenario.getRosbagTopic().c_str());
        rosTimeOffset->setText(QString::number(m_scenario.getRosbagOffset()));
        ifs.close();
    });

    QPushButton * saveScenario = new QPushButton();
    saveScenario->setText("Save Scenario");
    mainLayout->addWidget(saveScenario);
    connect(saveScenario, &QPushButton::pressed, [this](){
        string contents = Serializer::serialize_yaml(&m_scenario);
        QString name = QFileDialog::getSaveFileName(this, tr("Save Scenario"), "/home/scenario.yaml", tr("Scenarios (*.yaml)"));
        ofstream ofs(name.toStdString());
        ofs << contents << endl;
        ofs.close();
    });
}
