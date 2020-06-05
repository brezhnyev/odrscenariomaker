#include "PPPScene.h"

#include <iostream>
#include <condition_variable>
#include <mutex>
#include <signal.h>

using namespace std;
using namespace std::chrono_literals;

mutex mtx;
condition_variable cv;
bool isStopped;

void sighandler(int sig)
{
    switch(sig)
    {
        case SIGINT:
        if (isStopped)
        {
            cout << "\n***Handling Ctrl+C signal is in progress. Wait.***" << endl;
        }
        else
        {
            cout << "\n***Handling Ctrl+C signal...***" << endl;
            cout.flush();
            if (!isStopped)
            isStopped = true;
            cv.notify_all();
        }
        break;
    }
}

int main(int argc, const char *argv[])
{
    isStopped = false;
    signal(SIGINT, sighandler);

    if (argc < 1)
    {
        cout << "***Specify the configuration file.***" << endl;
        return 0;
    }
    PPPScene scene("../configuration.yaml");
    if (!scene.isInitialized()) return 1;

    thread keyboard(
        [&]()
        {
            bool isRecording = false;
            while (true)
            {
                char key;
                cin >> key;
                if (key == 'r')
                {
                    isRecording = !isRecording;
                    if (isRecording) cout << "Recording is started!" << endl;
                    else cout << "Recording is stopped!" << endl;
                    cout.flush();
                    scene.doRecording(isRecording);
                }
            }
        }
    );
    keyboard.detach(); // this is safe, since only waiting for key input. The thread will be destroyed on exit.

    scene.start();
    {
        unique_lock<mutex> lk(mtx);
        // MAIN THREAD: wait for Ctrl+C signal:
        cv.wait(lk, [](){ return isStopped; });
    }

    // If quitting lasts longer than 10 seconds, then abnormally quit
    // This can happen ex. when the server is closed/crashed/down
    bool doGracefulQuit = false;
    thread quittimer([&]()
    {
        unique_lock<mutex> lk(mtx);
        cv.wait_for(lk, 10s, [&](){ return doGracefulQuit; });
        if (!doGracefulQuit)
        {
            scene.stop(true);
            return 1;
        }
    });
    scene.stop(false); // if this lasts longer than 10 seconds the quittimer will fire and exit
    doGracefulQuit = true;
    cv.notify_all();
    quittimer.join();

    return 0;
}
