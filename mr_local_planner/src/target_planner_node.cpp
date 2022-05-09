#include "target_planner_node.h"
#include "ros/ros.h"


int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "target_planner");

    ros::NodeHandle n;
    ros::Subscriber target_sub = n.subscribe("/move_base_simple/goal", 1000, callback_goal);

    ros::Rate rate(10);


    while (ros::ok()) {
        /// calls your loop
        TargetPlannerNode::move();

        /// sets and publishes velocity commands
        TargetPlannerNode::publishMotion();


        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;

    return 0;
}

void callback_goal(const geometry_msgs::PoseStamped &goal) {
    goal_pose_ = goal;

    // ROS_INFO("goal received! %4.3f,%4.3f", goal_.x(), goal_.y());
}


void TargetPlannerNode::publishMotion() {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish(cmd);
}