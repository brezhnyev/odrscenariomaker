#include "PPPScene.h"

#include <iostream>
#include <condition_variable>
#include <mutex>
#include <signal.h>

using namespace std;

mutex mtx;
condition_variable cv;
bool isStopped;

void sighandler(int sig)
{
    switch(sig)
    {
        case SIGINT:
        cout << "\nGracefully quitting the client" << endl;
        isStopped = true;
        cv.notify_all();
        break;
    }
}

int main(int argc, const char *argv[])
{
    isStopped = false;
    signal(SIGINT, sighandler);

    if (argc < 2)
    {
        cout << "Specify the configuration file" << endl;
        return 0;
    }
    PPPScene scene(argv[1]);
    if (!scene.isInitialized()) return 1;
    scene.start();
    unique_lock<mutex> lk(mtx);
    cv.wait(lk, [](){ return isStopped; });
    scene.stop();

    return 0;
}
