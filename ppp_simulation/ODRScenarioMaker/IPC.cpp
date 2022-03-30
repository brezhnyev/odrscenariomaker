#include "IPC.h"
#include "scenario.h"

#include <QtWidgets/QMessageBox>

namespace net
{
    #include <sys/socket.h>
    #include <arpa/inet.h>
}
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <stdio.h>

#define PORT 12345 

using namespace std;


IPC::IPC(Scenario & scenario) : m_scenario(scenario), m_isconnected(false), m_sock(0), m_commThread(nullptr) {}

void IPC::slot_play()
{
    string data = m_serializer.serialize_yaml(&m_scenario);
    string command = "./client/client \"" + data + "\" &";
    system(command.c_str());

    usleep(1e6);

    if (m_commThread && !m_commThread->joinable()) delete m_commThread;

    m_commThread = new std::thread(
        [&, this]()
        {
            listenForResponse();
        }
    );
    m_commThread->detach();
}

void IPC::listenForResponse()
{
    using namespace net;

    m_sock = 0;
    static struct sockaddr_in serv_addr;

    if ((m_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    { 
        printf("\n Socket creation error \n");
        return; 
    } 
   
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)
    { 
        printf("\nInvalid address/ Address not supported \n");
        return; 
    }

    if (net::connect(m_sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    { 
        printf("\nConnection Failed \n");
        return;
    }

    while (true)
    {
        char buffer[1024] = {0};

        int valread = read( m_sock , buffer, 1024);
        if (valread == 1 && buffer[0] == '*') break;
        if (!valread) return;
        stringstream ss(buffer);
        int id; ss >> id;
        float x; ss >> x;
        float y; ss >> y;
        float z; ss >> z;
        float yaw; ss >> yaw;
        Actor * actor = dynamic_cast<Actor*>(m_scenario.children()[id]);
        if (actor) actor->setTrf(x,-y,z, 0, 0, -yaw);
        emit signal_update();
    }

    shutdown(m_sock, SHUT_RDWR);
}

void IPC::slot_stop()
{
    shutdown(m_sock, net::SHUT_RDWR);
}