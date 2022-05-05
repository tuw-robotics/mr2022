# mr2022
## Team members
* Daniel
* Luigi
* Felix
* Andreas

## Documentation
### basic startup
1. `roslaunch stage_ros world.launch world:=line.world`
2. `roslaunch mr_launch goto.launch`

### mock for GlobalPlanner
1. Set `2D Pose Estimate` using RViz
2. Set `2D Nav Goal` using RViz
3. You should see the fake-path in purple

### local planning with PurePursuit
1. Set the path as previous point.
2. You should see the next waypoint in green