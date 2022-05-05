#include "global_planner_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mr_global_planner");
    ros::NodeHandle n;
    GlobalPlannerNode planner(n);
    ros::Rate rate(3);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

GlobalPlannerNode::GlobalPlannerNode(ros::NodeHandle &n)
    : GlobalPlanner(), n_(n)
{
    this->sub_map_ = n.subscribe("map", 1, &GlobalPlannerNode::callbackMap, this);
    this->sub_goal_ = n.subscribe("move_base_simple/goal", 1, &GlobalPlannerNode::callbackGoal, this);

    this->pub_path_ = n.advertise<nav_msgs::Path>("global_planner/path", 1);
    this->pub_costmap_ = n.advertise<nav_msgs::OccupancyGrid>("global_planner/costmap", 1);
}

void GlobalPlannerNode::callbackMap(const nav_msgs::OccupancyGrid &occupancyGrid) {
    this->map_height = occupancyGrid.info.height;
    this->map_width = occupancyGrid.info.width;
    this->map_resolution_ = occupancyGrid.info.resolution;
    this->map_origin_ = occupancyGrid.info.origin;

    for (size_t i = 0; i < occupancyGrid.data.size(); i++) {
        moro::Cell c = {};
        c.x = i % occupancyGrid.info.width;
        c.y = i / occupancyGrid.info.height;

        this->map.insert({c, occupancyGrid.data.at(i)});
    }
}

void GlobalPlannerNode::callbackGoal(const geometry_msgs::PoseStamped &goal) {
    // Transform input coordinates into map / cell coordinates.
    this->start.x = 100;
    this->start.y = 100;
    this->goal.x = 400;
    this->goal.y = 400;

    // Plant
    this->plan();

    this->publish_costmap();
    this->publish_path();
}

/**
 * Copies the data from the internal costmap into a ROS format.
 *
 * This normalizes the costs, such that they are between 0 and 100.
 * -1 stands for unknown cost.
 */
void GlobalPlannerNode::publish_costmap() {
    std::vector<int8_t> costmap(this->map_height * this->map_width, -1);
    double max = 0;

    for (size_t i = 0; i < this->map_width * this->map_height; i++) {
        moro::Cell c = {};
        c.x = i % this->map_width;
        c.y = i / this->map_height;

        if (this->cost.count(c)) {
            double cost = this->cost.at(c);

            if (cost > max) {
                max = cost;
            }
        }
    }

    for (size_t i = 0; i < this->map_width * this->map_height; i++) {
        moro::Cell c = {};
        c.x = i % this->map_width;
        c.y = i / this->map_height;

        if (this->cost.count(c) && this->cost.at(c) != -1) {
            double cost = 100 - this->cost.at(c) / max * 100;
            costmap.at(i) = cost;
        }
    }

    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.info.height = this->map_height;
    msg.info.width = this->map_width;
    msg.info.origin = this->map_origin_;
    msg.info.resolution = this->map_resolution_;
    msg.info.map_load_time = ros::Time::now();
    msg.data = costmap;

    this->pub_costmap_.publish(msg);
}

void GlobalPlannerNode::publish_path() {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();

    for (moro::Cell &c : this->path) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = ros::Time::now();

        p.pose.position.x = c.x * this->map_resolution_ + this->map_origin_.position.x;
        p.pose.position.y = c.y * this->map_resolution_ + this->map_origin_.position.y;

        path_msg.poses.push_back(p);
    }

    this->pub_path_.publish(path_msg);
}
