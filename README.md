# mr2022
## Team members

Philipp-Alexander Auer, Patrik Szabó

## 0. `git merge` - `[20/20]`

Participants: Philipp-Alexander Auer, Patrik Szabó

While using the version control software `git` we used branches to develop individual features/tasks and used `git merge` as can be seen in our `git log`.

## 1. Publish the used map - `[45/45]`

Participants: Philipp-Alexander Auer, Patrik Szabó

![map_rviz](images/map-rviz.png)

## 2. Initialize self-localization and trigger driving using RViz - `[50/50]`

Sending messages using `ROS_DEBUG` can be e.g. seen using `rqt_console`:

![debug_console](images/debug-console.png)

## 3. Connect self-localization and planner - `[45/45]`

By using a TransformBroadcaster inside the self_localization_node, we were able to publish the transform between the map frame and the odometry frame. This broadcasted message was then "caught" using a listener in the local_planner_node. The image shows the tf-tree connection between the map and odom frames. 

![tf tree](images/tf-tree.png)

## 4. Planner

We connected Rviz to the planner, by "posting" a goal pose on the topic move_base_simple/goal. This triggers a callback in the local_planner_node, which in turn saves the position passed down from rviz inside a goal_ variable. This variable is used in the planner node, to guide the robot towards the selected point on the map. Our approach consists of calculating the angle between the direction the robot is facing, and the goal point. The robot is then turned to face the goal, and starts driving in a straight line. We use a simple iteration "timer" to counteract the oversteering that happens, when the angular velocity hasn't been fully reset yet. When a new goal is set, the driving resets, and a new heading angle is calculated. 
