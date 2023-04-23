# Building ODRScenariomaker
The ODRScenario maker is a GUI application for building scenarios that can run in Carla. The application consists of two parts: GUI and the carla-client part. The two are communicating via TCP/IP. This approach has cons and pros, however main reason to split was impossibility to compile Qt and Carla-related classes in one project.  
The Serializer is the class shared by two parts of the application so it is made a library for the client to use it.  
The same issues hold with the soft links: for the Carla-client there should be prepared two folders *include* and *lib*. Here is how the files should look like:  
<pre>
.
├── Actor.cpp
├── Actor.h
├── ActorProps.cpp
├── ActorProps.h
├── Canvas.cpp
├── Canvas.h
├── client
│   ├── CMakeLists.txt
│   ├── include
│   ├── lib
│   └── play.cpp
├── CMakeLists.txt
├── data
│   └── Town02.jpg
├── IPC.cpp
├── IPC.h
├── main.cpp
├── MainWindow.cpp
├── MainWindow.h
├── ODRScenarioMaker
├── release
│   ├── client
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── libser.a
│   ├── Makefile
│   ├── ODRScenarioMaker
│   ├── ODRScenarioMaker_autogen
│   └── ser_autogen
├── scenario.cpp
├── scenario.h
├── ScenarioProps.cpp
├── ScenarioProps.h
├── Selectable.cpp
├── Selectable.h
├── Serializer.cpp
...
</pre>

# Starting ODRScenarioMaker with standalone Carla Engine
The ODRScenarioMaker is started in a standard was as a standalone application. At this point there is no need to have Carla Engine running, since the ODRScenarioMaker will not need carla during the setup/edditing.  
Select the scenario (Scenario 0) and "Add Vehicle" on the right panel:
![Add Vehicle](./images/Add_Vehicle.jpg)  
The Vehicle will be displayed near to the start of the Coordinate System (maybe far away from where the Town map is). Click "A" on the keyboard and zoom in/out to find it:  
![Vehicle location](./images/Vehicle_location.jpg)  
Add Waypath for the selected Vehicle. Select the newly created Waypath in the tree. Now keeping *Shift* pressed the waypoints may be added (usually along the road):  
![Add waypoints](./images/Add_waypoints.jpg)  

Another car maybe added with waypaths/waypoints. (Only one waypath will be played back now per vehicle)

After the scenario is prepared the carla engine should be started. Go to the folder with Carla and start Carla with  
**./CarlaEU4.sh**   
The look of Carla on the first start:

![Before play](./images/before_play.jpg)  
Another possible view (after first start is complete). Here the properly loaded Map is displayed:  
![Before play2](./images/before_play2.jpg)  

Press **Play**. The two cars (Audi and VW mini bus) will move in opposite directions making two curves: 

![During play](./images/during_play.jpg)  

**KNOWN ISSUES**: closing the ODRScenarioMaker not always closes the TCP connection. So check the "client" application in system to avoid dozens of open TCP connections:

![client system heck](./images/system_check_client.jpg)  


# Starting ODRScenarioMaker with UE4Editor
Very similar to the previous procedure, just instead of the standalone Carla the Carla-project should be started as mentioned in the "Legacy Way" from the "carla" folder:  
**make launch-only**
Press "Play" to start the server inside the UE4Editor:  
![playing_in_UE4Editor](./images/playing_in_UE4Editor.jpg)  
Start "play" in the ODRScenarioMaker:  
![playing_ODR_UE4Editor](./images/playing_ODR_UE4Editor.jpg)  
