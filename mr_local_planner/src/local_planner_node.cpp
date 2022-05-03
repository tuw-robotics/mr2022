#include "local_planner_node.h"
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace moro;

int main(int argc, char **argv) {

    ros::init(argc, argv, "planner_local");  /// initializes the ros node with default name
    ros::NodeHandle n;
    LocalPlannerNode planner(n);
    planner.init();
    ros::Rate rate(10);  /// ros loop frequency synchronized with the wall time (simulated time)

    while (ros::ok()) {

        /// calls your loop
        planner.ai();

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// plots measurements
        planner.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/**
 * Constructor
 **/
LocalPlannerNode::LocalPlannerNode(ros::NodeHandle &n)
        : LocalPlanner(ros::NodeHandle("~").getNamespace()),
          n_(n),
          n_param_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_) {

#if PLANNER_EXERCISE >= 10
#else
    this->sub_laser_ = n.subscribe("scan", 1000, &LocalPlannerNode::callbackLaser, this);
#endif

#if PLANNER_EXERCISE >= 10
#else
    this->sub_goal_ = n.subscribe("goal", 1000, &LocalPlannerNode::callbackGoal, this);
    this->sub_odom_ = n.subscribe("odom", 1000, &LocalPlannerNode::callbackOdometry, this);
#endif

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    reconfigureFnc_ = boost::bind(&LocalPlannerNode::callbackConfigLocalPlanner, this, _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);
}

void LocalPlannerNode::callbackConfigLocalPlanner(mr_local_planner::LocalPlannerConfig &config, uint32_t level) {
    ROS_INFO ("callbackConfigLocalPlanner!");
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void LocalPlannerNode::callbackLaser(const sensor_msgs::LaserScan &_laser) {
#if PLANNER_EXERCISE >= 20
#else
    std::string ns = this->n_.getNamespace().substr(1);
    std::string source = "base_laser_link";;
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
#endif
}

/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void LocalPlannerNode::callbackOdometry(const nav_msgs::Odometry &odom) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double a = yaw;
    odom_.set(odom.pose.pose.position.x, odom.pose.pose.position.y, a);
    odom_.recompute_cached_cos_sin();
}


/**
 * copies incoming pose messages to the base class
 * @param odom
 **/
void LocalPlannerNode::callbackGoal(const geometry_msgs::Pose2D &goal) {
    goal_.set(goal.x, goal.y, goal.theta);
    goal_.recompute_cached_cos_sin();

    start_ = odom_;
    action_state_ = ActionState::INIT;

    Point2D goal_local;
    ROS_INFO ("goal received! %4.3f,%4.3f", goal_.x(), goal_.y());
}


/**
 * Publishes motion commands for a robot
 **/
void LocalPlannerNode::publishMotion() {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish(cmd);
}
