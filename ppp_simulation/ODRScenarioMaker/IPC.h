#pragma once

#include <thread>
#include <QtCore/QObject>

#include "Serializer.h"

class Scenario;

class IPC : public QObject
{
    Q_OBJECT
public:
    IPC(Scenario & scenario);

signals:
    void signal_update();

public slots:
    void slot_play();
    void slot_stop();

private:
    void listenForResponse();

private:
    Scenario &m_scenario;
    bool            m_isconnected;
    int             m_sock;
    std::thread *   m_commThread;
    Serializer      m_serializer;
};