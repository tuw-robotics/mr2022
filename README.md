# mr2022
## Team members

* Schmidt, Franz
* Windischbauer, Johannes

## Documentation

### 1. Publish the used map (45p)
The used maps are being published by the map_server node. To be able to utilize these maps I created the necessary yaml files and moved everything into mr_launch/cfg/maps.
The map_server node is included in the mr_goto/launch/goto.launch file and the maps are configurable with the map argument. The default map is line.

`roslaunch stage_ros world.launch world:=line.world`
`roslaunch mr_goto goto.launch map:=line.yaml`

### 4. Planner
#### 4.1 New Node

The mr_goto node is a new python node and already connects to the scan messages and publishes velocity commands. At the moment the driving is overcomplicated and can possibly be simplified quite easily.
