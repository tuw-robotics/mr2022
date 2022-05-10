#include "target_planner_node.h"
#include "ros/ros.h"

geometry_msgs::Pose2D target_pose;
geometry_msgs::Pose2D current_pose;

void callback_target(const geometry_msgs::PoseStamped &goal) {
    target_pose.x = goal.pose.position.x;
    target_pose.y = goal.pose.position.y;
    target_pose.theta = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

    ROS_DEBUG("received message on /move_base_simple/goal");
}

void callback_pose(const geometry_msgs::PoseWithCovarianceStamped &pose) {
    current_pose.x = pose.pose.pose.position.x;
    current_pose.y = pose.pose.pose.position.y;
    current_pose.theta = pose.pose.pose.orientation.w;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "target_planner");

    ros::NodeHandle n;
    TargetPlannerNode planner(n);
    ros::Subscriber target_sub = n.subscribe("/move_base_simple/goal", 1000, callback_target);
    ros::Subscriber pose_sub = n.subscribe("/pose_estimated", 1000, callback_pose);


    ros::Rate rate(10);

    while (ros::ok()) {
        /// calls your loop
        planner.move();


        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }

    return 0;
}


TargetPlannerNode::TargetPlannerNode(ros::NodeHandle &n) : TargetPlanner(), n_(n), n_param_("~") {

    pub_cmd_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    tf_listener_ = std::make_shared<tf::TransformListener>();
}

void TargetPlannerNode::move() {
    double dx = 0;
    double dy = 0;
    double angle_diff = 0;

    tf::StampedTransform transform;
    if (tf_listener_->frameExists("/odom")) {
        tf_listener_->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    }

    dx = target_pose.x - transform.getOrigin().x();
    dy = target_pose.y - transform.getOrigin().y();
    double target_angle = std::atan2(dy, dx);
    auto rotation = transform.getRotation();
    auto current_angle = 2 * atan2(rotation.z(), rotation.w());
    angle_diff = moro::angle_difference(moro::angle_normalize(target_angle), moro::angle_normalize(current_angle));

    // Alternatively use pose_estimated topic for current position
    /*
    dx = target_pose.x - current_pose.x;
    dy = target_pose.y - current_pose.y;
    double target_angle = std::atan2(dy, dx);
    angle_diff = moro::angle_difference(moro::angle_normalize(target_angle), moro::angle_normalize(current_pose.theta));
    */

    double pos_diff = sqrt(dx * dx + dy * dy);


    double vel = 0.0;
    double rot = 0.0;
    if (pos_diff >= 0.25) {
        vel = 0.2;

        if (angle_diff > 0.3) {
            rot = 0.4;
        } else if (angle_diff < -0.3) {
            rot = -0.4;
        } else if (angle_diff > 0.1) {
            rot = 0.2;
        } else if (angle_diff < -0.1) {
            rot = -0.2;
        } else {
            vel = 0.5;
        }
    } else { // position is reached -> turn into target orientation
        double target_angle_diff = moro::angle_difference(moro::angle_normalize(target_pose.theta), moro::angle_normalize(current_pose.theta));

        if (target_angle_diff > 0.08) {
            rot = 0.15;
        } else if (target_angle_diff < -0.08) {
            rot = -0.15;
        }
    }


    geometry_msgs::Twist cmd;
    cmd.linear.x = vel;
    cmd.angular.z = rot;
    pub_cmd_.publish(cmd);
}