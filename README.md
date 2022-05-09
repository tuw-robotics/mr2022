# mr2022
## Team members
Georg Amstetter, Menno Haász, Jan Hadl, Florian Pressel

## Documentation

### 1. Publish the used map

We managed to publish the map which was used by the self localization node to RViz.
The result can be seen in the following figure.

![Published map to RViz](./images/rviz_map.png)

### 2. Initialize self-localization and trigger driving using RViz

We managed to subscribe to **/move_base_simple/goal** and **/initialpose** in the respective nodes, interpret
the received messages and log the received poses. 

### 3. Connect self-localization and planner

We managed to broadcast and receive the robots estimated pose as TF in the respective nodes.
The broadcasted TF can be seen in the following figure.

![Broadcasted estimated pose TF](./images/pose_estimated_rviz.png)

### 4. Planner

DONE

#### 4.1 New Node

DONE: mr_goto (mode 1)

#### 4.2 Simple, no Obstacle

DONE

#### 4.3 Avoid obstacle

TODO

#### 4.4 Plan

DONE

## Achieved Points

|                              Task                              | Points |
|:--------------------------------------------------------------:|:------:|
|                     1. Publish the used map                    |   45   |
| 2. Initialize self-localization and trigger driving using RViz |   50   |
|            3. Connect self-localization and planner            |   45   |
|                           4. Planner                           |   40   |
|                          4.1 New Node                          |   50   |
|                     4.2 Simple, no Obstacle                    |   50   |
|                       4.3 Avoid obstacle                       |   50   |
|                            4.4 Plan                            |   100  |
|                            GIT merge                           |   20   |
|                              Total                             |   450  |