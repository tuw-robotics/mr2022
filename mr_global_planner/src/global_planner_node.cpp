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
    : GlobalPlanner(), n_(n), tf_buffer_(), tf_listener_(tf_buffer_)
{
    this->sub_map_ = n.subscribe("map", 1, &GlobalPlannerNode::callback_map, this);
    this->sub_goal_ = n.subscribe("move_base_simple/goal", 1, &GlobalPlannerNode::callback_goal, this);

    this->pub_path_ = n.advertise<nav_msgs::Path>("global_planner/path", 1);
    this->pub_costmap_ = n.advertise<nav_msgs::OccupancyGrid>("global_planner/costmap", 1);
}

void GlobalPlannerNode::callback_map(const nav_msgs::OccupancyGrid &occupancyGrid) {
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

void GlobalPlannerNode::callback_goal(const geometry_msgs::PoseStamped &target) {
    // Get & Set start position
    // TODO: Handle Timeout
    geometry_msgs::TransformStamped start_tf = this->tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(3.0));

    auto t1 = this->map_to_img({start_tf.transform.translation.x, start_tf.transform.translation.y});

    std::cout << "tf: " << start_tf.transform.translation.x << ":" << start_tf.transform.translation.y << std::endl;

    this->start.x = t1.first;
    this->start.y = t1.second;

    // Transform input coordinates into map / cell coordinates.
    auto t2 = this->map_to_img({target.pose.position.x, target.pose.position.y});

    this->goal.x = t2.first;
    this->goal.y = t2.second;

    std::cout << "Start: " << this->start.x << ":" << this->start.y << std::endl;
    std::cout << "Target: " << this->goal.x  << ":" << this->goal.y << std::endl;


    // Plan
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

        auto t = this->img_to_map({c.x, c.y});

        p.pose.position.x = t.first;
        p.pose.position.y = t.second;

        path_msg.poses.push_back(p);
    }

    this->pub_path_.publish(path_msg);
}

std::pair<size_t, size_t> GlobalPlannerNode::map_to_img(std::pair<double, double> p) {
    return {
            (p.first - this->map_origin_.position.x) / this->map_resolution_,
            (p.second - this->map_origin_.position.y) /  this->map_resolution_
    };
}

std::pair<double, double> GlobalPlannerNode::img_to_map(std::pair<size_t, size_t> p) {
    return {
            p.first * this->map_resolution_ + this->map_origin_.position.x,
            p.second * this->map_resolution_ + this->map_origin_.position.y
    };
}
