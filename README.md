# mr2022
## Team members

* Schmidt, Franz
* Windischbauer, Johannes

## Documentation


### 0. System launch
The following launches will start all necessary nodes and also opens RViz with a predefined setup for visualization.

1. `roslaunch stage_ros world.launch world:=line.world`
2. `roslaunch mr_goto goto.launch map:=line.yaml`


### 1. Publish the used map (45p)
The used maps are being published by the map_server node. To be able to utilize these maps I created the necessary yaml files and moved everything into mr_launch/cfg/maps.
The map_server node is included in the mr_goto/launch/goto.launch file and the maps are configurable with the map argument. The default map is line.


### 2. Initialize self-localization and trigger driving using RViz (50p)
The self-localization node subscribes to the `/initialpose` topic and listenes for the event which can be triggered with RViz. The local-planner node subscribes to the `/move_base_simple/goal` topic which as well can be triggered with Rviz.

![image-20220510173730920](./img/image-20220510173730920.png)
![image-20220510173730920](./img/image-20220510173730920.png)


### 3. Connect self-localization and planner (45p)
The planner repeatingly updates the transform for the robot pose which is provided by the self-localization node. The driving can be triggered from RViz by setting the goal-position. Also the TF tree can be visualized to see the complete transformation from map to base-link.


### 4. Planner
#### 4.1 New Node (50p)

The mr_goto node is a new python node and already connects to the scan messages and publishes velocity commands. At the moment the driving is overcomplicated and can possibly be simplified quite easily.


### 5. Using git to merge project






