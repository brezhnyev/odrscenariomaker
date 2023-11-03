#ifndef CARLACLIENT_H
#define CARLACLIENT_H

#include "scenario.h"
#include <condition_variable>
#include <mutex>

class MainWindow;

class Client
{
public:
  enum ePlayStatus { STOP, PAUSE, PLAY };
  Client(MainWindow * w) : m_window(w) {}
  virtual void play(Scenario & scenario)=0;
  virtual void playDummy(Scenario & scenario)=0;

  ADDVAR(protected, ePlayStatus, playStatus, STOP);
  ADDVRR(protected, Eigen::Matrix4f, camTrf, Eigen::Matrix4f());
  ADDVAR(protected, MainWindow *, window, nullptr);
  ADDVAR(protected, int, FPS, 10);
  ADDVAR(protected, bool, realtimePlayback, true);
  ADDVAR(protected, bool, isSynchronous, true);

  std::condition_variable & get_playCondVar() { return m_playCondVar; }
  std::mutex & get_playCondVarMtx() { return m_playCondVarMtx; }

protected:
  std::condition_variable m_playCondVar;
  std::mutex m_playCondVarMtx;
};

class CarlaClient : public Client
{
public:
  CarlaClient(MainWindow * w) : Client(w) {}
  void play(Scenario & scenario) override;
  void playDummy(Scenario & scenario) override;
};


#endif