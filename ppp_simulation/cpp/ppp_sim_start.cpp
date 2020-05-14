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
    // wait for Ctrl+C signal:
    cv.wait(lk, [](){ return isStopped; });

    cout << "Waiting 10 seconds to stop the client (ensure last callbacks are processed)." << endl;
    // If quitting lasts longer than 10 seconds, then ubnormally quit
    // This can happen ex. when the server is closed/crashed
    bool doGraceFulQuit = false;
    thread t([&]()
    { 
        sleep(10); 
        if (!doGraceFulQuit) exit(1); 
    });
    scene.stop();
    doGraceFulQuit = true;
    t.join();

    return 0;
}
