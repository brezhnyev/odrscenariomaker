#ifndef CARLACLIENT_H
#define CARLACLIENT_H

#include "scenario.h"
#include <condition_variable>
#include <mutex>

template<typename T>
class Client
{
public:
  virtual void play(Scenario & scenario);

  ADDVAR(protected, int, playStatus, 0);
  ADDVAR(protected, Eigen::Matrix4f, camTrf, Eigen::Matrix4f());
  ADDVAR(protected, T*, window, nullptr);
  ADDVAR(protected, int, FPS, 10);
  ADDVAR(protected, bool, realtimePlayback, true);
  ADDVAR(protected, bool, isSynchronous, true);

  const std::condition_variable & getPlayCV() { return m_playCondVar; }
  const std::mutex & getPlayCVMtx() { return m_playCondVarMtx; }

protected:
  std::condition_variable m_playCondVar;
  std::mutex m_playCondVarMtx;
};

template<typename T>
class CarlaClient : public Client<T>
{
public:
    void play(Scenario & scenario);
};

#endif