# mr2022
## Team members
* Daniel
* Luigi
* Felix
* Andreas

## How to start

### Dependencies
```
scipy~=1.8.0
scikit-image~=0.19.2
pyyaml~=6.0
rospkg~=1.4.0
```

### Basic startup
1. `roslaunch stage_ros world.launch world:=cave.world`
2. `roslaunch mr_launch goto.launch`
* * optional: add argument `map:=cave` or `map:=line`


### How to use the GlobalPlanner
1. Set `2D Pose Estimate` using RViz
2. Set `2D Nav Goal` using RViz
3. You should see the path in purple

### local planning with PurePursuit
1. Set the path as previous point.
2. You should see the next waypoint in green

## Documentation
### Publish the used map. (45 Points)
1. Start like described in _Basic startup_.
2. Map as published on topic `/map` is shown in RViz. You can enable the topic `/map_ref` to compare with a version published by map_server.

### Initialize self-localization and trigger driving using RViz (50 Points)
1. Start like described in _Basic startup_.
2. Set `2D Pose Estimate` using RViz.
3. Set `2D Nav Goal` using RViz.
4. You will see the `2D Pose Estimate` visualized with a green arrow.
5. You will see the `2D Nav Goal` visualized with a red arrow.
7. Wait for some seconds and you will see the path from the global planner visualized in purple.

### Connect self-localization and planner (45 Points)
1. Start like described in _Basic startup_.
2. You will see the current pose visualized with a blue arrow.
3. You can additionally see the TF tree visualized.

### Planner
* Our planner is using our self-localization (20 Points):
* * TODO: We should be able to use `/pose_estimated` instead of `/odom`
* Your planner can be operated with Rviz (20 Points)
* * See section in _Initialize self-localization and trigger driving using RViz_.

#### New Node
* For implementing the planner in a newly created node, you will get a reward. (50 Points).
* * We have added separate nodes.

#### Simple, no Obstacle
* Your vehicle can drive to a goal location and stops there. (25 Points)
* * Working. How to use, see section in _Initialize self-localization and trigger driving using RViz_.
* Your vehicle can drive to a goal location, stops there and turns into the correct pose. (25 Points)
* * TODO: turn afterwards.

#### Avoid obstacle
* Your vehicle can drive to a goal location even if there is obstacle with 1x1m (movable box) in size in between. (25 Points)
* * TODO: ...
* Your vehicle can drive to a goal location even if there is a cave obstacle such the one [-5,-3] in between. (25 Points)
* * TODO: ...

#### Plan
* Write a node or modify the planner and/or self-localization to plan a path to the goal location using waypoints and publish it as ROS nav_msgs/Path message. (50 Points)
* * The path is planned and published to `/waypoints` by the global planner.
* Make the local planner to follow the nav_msgs/Path. (50 Points)
* * The path is followed by the local planner.

