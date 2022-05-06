#ifndef GLOBAL_PLANNER_NODE_H
#define GLOBAL_PLANNER_NODE_H

#include "mr_global_planner/global_planner.h"

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class GlobalPlannerNode : public moro::GlobalPlanner {
public:
    GlobalPlannerNode(ros::NodeHandle &n);

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_goal_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Publisher pub_path_;
    ros::Publisher pub_costmap_;
    float map_resolution_;
    geometry_msgs::Pose map_origin_;

    void callback_map(const nav_msgs::OccupancyGrid &);
    void callback_goal(const geometry_msgs::PoseStamped &);

    void publish_costmap();
    void publish_path();

    std::pair<size_t, size_t> map_to_img(std::pair<double, double>);
    std::pair<double, double> img_to_map(std::pair<size_t, size_t>);
};

#endif // GLOBAL_PLANNER_NODE_H
