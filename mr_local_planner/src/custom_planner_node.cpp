#include "custom_planner_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace moro;
int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "planner_custom" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    CustomPlannerNode planner ( n );
    ros::Rate rate ( 10 );  /// ros loop frequency synchronized with the wall time (simulated time)
    
    while ( ros::ok() ) {

        /// calls your loop
        planner.ai();

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleeps for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

/// Constructor
CustomPlannerNode::CustomPlannerNode ( ros::NodeHandle & n )
    : CustomPlanner ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

    sub_laser_ = n_.subscribe("scan", 1000, &CustomPlannerNode::callbackLaser, this);
    sub_goal_ = n_.subscribe("move_base_simple/goal", 1000, &CustomPlannerNode::callbackGoal, this);
    sub_odom_ = n_.subscribe("odom", 1000, &CustomPlannerNode::callbackOdometry, this);
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );
}

/// Listen to LaserScan messages
void CustomPlannerNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {
    measurement_laser_.range_max() = _laser.range_max;
    measurement_laser_.range_min() = _laser.range_min;
    measurement_laser_.resize(_laser.ranges.size());

    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        measurement_laser_ [i].length  = _laser.ranges[i];
        measurement_laser_ [i].angle  = _laser.angle_min + i*_laser.angle_increment;
        // The assignment mentions the laser to be mounted 22cm in front of the robot, no rotation.
        // The sensor-to-robot rotation matrix is omitted.
        double robotX = cos(measurement_laser_[i].angle) * measurement_laser_[i].length + .22f;
        double robotY = sin(measurement_laser_[i].angle) * measurement_laser_[i].length;
        Polar2D polar(measurement_laser_ [i].angle, measurement_laser_ [i].length);
        measurement_laser_ [i].end_point = Point2D(robotX, robotY);
    }
    
    /// Listen for the latest incoming broadcast and save the transform inside tf_
    try{
        tf_listener_.lookupTransform("odom", "map", ros::Time(0), tf_);
    }catch(tf::LookupException e){
        ROS_INFO ("%s", e.what());
    }
}

/// Listen to Odometry messages
void CustomPlannerNode::callbackOdometry ( const nav_msgs::Odometry &odom ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( odom.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    odom_.set ( odom.pose.pose.position.x, odom.pose.pose.position.y, a );
    odom_.recompute_cached_cos_sin();
}

/// Listen to goal messages
void CustomPlannerNode::callbackGoal ( const geometry_msgs::PoseStamped& goal ) {
    goal_.set ( goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w );
    goal_.recompute_cached_cos_sin();

    start_ = odom_;
    action_state_ = ActionState::INIT;

    Point2D goal_local;
    ROS_INFO ( "goal received! %4.3f,%4.3f",  goal_.x(), goal_.y() );
    ROS_DEBUG ( "callbackGoal!");
}

/// Publish resulting motion commands
void CustomPlannerNode::publishMotion () {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish ( cmd );
}
