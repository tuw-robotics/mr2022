#include "planner_node.h"
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace moro;

int main(int argc, char **argv) {

    ros::init(argc, argv, "planner");  /// initializes the ros node with default name
    ros::NodeHandle n;
    PlannerNode planner(n);
    planner.init();
    ros::Rate rate(10);  /// ros loop frequency synchronized with the wall time (simulated time)

    while (ros::ok()) {

        planner.updateEstimatedPose();
        planner.ai(planner.estimatedPose());

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

PlannerNode::PlannerNode(ros::NodeHandle &n)
        : Planner(ros::NodeHandle("~").getNamespace()),
          n_(n),
          n_param_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_) {

    //Subscribes to scan for laser scans and goal for modes STRAIGHT and STRAIGHT_AVOID.
    this->sub_laser_ = n.subscribe("scan", 1000, &PlannerNode::callbackLaser, this);
    this->sub_goal_ = n.subscribe("goal", 1, &PlannerNode::callbackGoal, this);

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    reconfigureFnc_ = boost::bind(&PlannerNode::callbackConfigPlanner, this, _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void PlannerNode::callbackConfigPlanner(mr_planner::PlannerConfig &config, uint32_t level) {
    ROS_INFO ("callbackConfigPlanner!");
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void PlannerNode::callbackLaser(const sensor_msgs::LaserScan &_laser) {

    std::string ns = this->n_.getNamespace().substr(1);
    std::string source = "base_laser_link";
    std::string target = "base_link";
    geometry_msgs::TransformStamped scanner_transform = tf_buffer_.lookupTransform(target, source, ros::Time(0));

    measurement_laser_.range_max() = _laser.range_max;
    measurement_laser_.range_min() = _laser.range_max;
    measurement_laser_.resize(_laser.ranges.size());

    for (int i = 0; i < _laser.ranges.size(); ++i) {
        measurement_laser_[i].length = _laser.ranges[i];
        measurement_laser_[i].angle = _laser.angle_min + (i * _laser.angle_increment);

        double x = scanner_transform.transform.translation.x + cos(measurement_laser_[i].angle) * measurement_laser_[i].length;
        double y = scanner_transform.transform.translation.y + sin(measurement_laser_[i].angle) * measurement_laser_[i].length;
        measurement_laser_[i].end_point.set(x, y);
    }
}

/*
* @ param the goal to reach in world coordinates. 
*/

void PlannerNode::callbackGoal(const geometry_msgs::PoseStamped &goal) {
    ROS_INFO ("goal received! %4.3f,%4.3f", goal_.x(), goal_.y());

    goal_.set(goal.pose.position.x, goal.pose.position.y, 0);
    goal_.recompute_cached_cos_sin();

    start_ = estimatedPose();

    action_state_ = ActionState::INIT;

}


/**
 * Publishes motion commands for a robot
 **/
void PlannerNode::publishMotion() {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish(cmd);
}

Pose2D PlannerNode::estimatedPose(){
    return pose_estimation_;
}

void PlannerNode::updateEstimatedPose(){
    try{
    geometry_msgs::TransformStamped start_tf = this->tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(3.0));

    pose_estimation_ = Pose2D(
        start_tf.transform.translation.x,
        start_tf.transform.translation.y,
        start_tf.transform.rotation.z);
    
    }catch(tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
 }