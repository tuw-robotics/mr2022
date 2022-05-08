# mr2022

## Team members

* Gerhard Jungwirth
* Lisa-Maria Kipfer
* Nino Wegleitner

## 1. Publish the used map (45 Points)
by Nino Wegleitner

:--------------------:|:--------------------:
![](img/map_rviz.png) | ![](img/map_rviz2.png.png)

The map was published using a timer callback function. 

## 2. Initialize self-localization and trigger driving using RViz (50 Points)
by Nino Wegleitner

* SCREENSHOT OF DEBUG MSGS

## 3. Connect self-localization and planner (45 Points)
by Nino Wegleitner

Tf from map to base_link was already broadcasted in earlier exercise. A tf listener was implemented in the local planner node. A stamped transformation is saved every time the laser callback function is called.   
