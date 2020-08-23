# Alternatives to using RoadRunner for creation of XODR and MAPs for Carla

## Using open street maps as basis for Carla Maps

Open Street Maps provide the option "Export" the visible or selected area. The exported file has .osm extension and usually stores the primitive 3D objects with or without texture coordinates and the road metainformation (directions, lanes, junctions, etc):  
![osm map](./images/osm_map.jpg)  

One of the possibilities to view the exported osm file is to use OSM2World Viewer (http://osm2world.org/download/):  
<pre>
./osm2world.sh --gui --config standard.properties
</pre>
![osm2worlviewer](./images/osm2worlviewer.jpg)  

Edditing the OSM files is possible with some editors, ex. netedit from SUMO:
<pre>./netedit</pre>
![sumo-netedit](./images/sumo-netedit.jpg)  

Here is a list of OSM editors:  
https://wiki.openstreetmap.org/wiki/Comparison_of_editors  

![josm](./images/josm.jpg)  

The OSM2World viewer allows exporting the scene into .obj file/folder. Good idea to export the scene into a folder, then all files of the scene will be in one isolated folder:  
![exportOSM2Folder](./images/exportOSM2Folder.jpg)  
The OBJ file can be imported into the UE4Editor to be refined/modified.
 
![importing_obj_file](./images/importing_obj_file.jpg)  
![rotate_and_pos](./images/rotate_and_pos.jpg)  
![final_import](./images/final_import.jpg)  

**It is at the moment not quite clear how to make/modify Maps to use in Carla simulator. Apparently the only proper way is to open the appropriate .uproject file.**

Extra references:  
* https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html
* https://github.com/carla-simulator/carla/issues/1998
* https://github.com/carla-simulator/carla/issues/1015
* https://docs.unrealengine.com/en-US/Engine/Landscape/Creation/index.html?utm_source=editor&utm_medium=docs&utm_campaign=tutorials
* https://sumo.dlr.de/docs/netedit.html
* https://sumo.dlr.de/docs/netconvert.html