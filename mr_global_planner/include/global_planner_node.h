#ifndef GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_NODE_H

#include "mr_global_planner/global_planner.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


class GlobalPlannerNode : public moro::GlobalPlanner {
public:
    GlobalPlannerNode(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_goal_;
    ros::Publisher pub_path_;
    ros::Publisher pub_costmap_;
    float map_resolution_;
    geometry_msgs::Pose map_origin_;

    void callbackMap(const nav_msgs::OccupancyGrid &);
    void callbackGoal(const geometry_msgs::PoseStamped &);

    void publish_costmap();
    void publish_path();
};

#endif // GLOBAL_PLANNER_NODE_H
