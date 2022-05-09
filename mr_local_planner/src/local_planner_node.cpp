#include "local_planner_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace moro;
int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "planner_local" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    LocalPlannerNode planner ( n );
    planner.init();
    ros::Rate rate ( 10 );  /// ros loop frequency synchronized with the wall time (simulated time)

    srand(time(0));
    
    while ( ros::ok() ) {

        /// calls your loop
        planner.ai();

        /// sets and publishes velocity commands
        planner.publishMotion();

        /// visualition
        planner.visualization();

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
LocalPlannerNode::LocalPlannerNode ( ros::NodeHandle & n )
    : LocalPlanner ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

    /**
     * @ToDo Wanderer
     * @see http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
     * subscribes the fnc callbackLaser to a topic called "scan"
     * since the simulation is not providing a scan topic /base_scan has to be remapped
     **/
#if PLANNER_EXERCISE >= 10
#else
    /**
     * @node your code
     **/
    sub_laser_ = n_.subscribe("scan", 1000, &LocalPlannerNode::callbackLaser, this);
#endif


    /**
     * @ToDo GoTo
     * subscribes the fnc callbackGoal to goal and fnc callbackOdometry to odom
     **/
#if PLANNER_EXERCISE >= 10
#else
    /**
     * @node your code
     **/
    sub_goal_ = n_.subscribe("goal", 1000, &LocalPlannerNode::callbackGoal, this);
    sub_odom_ = n_.subscribe("odom", 1000, &LocalPlannerNode::callbackOdometry, this);
    sub_path_ = n_.subscribe("waypoints", 1000, &LocalPlannerNode::callbackPath, this);
#endif

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

    /// defines a publisher for the next waypoint in pure pursuit
    pub_waypoint_ = n.advertise<visualization_msgs::Marker> ( "next_waypoint", 1 );

    reconfigureFnc_ = boost::bind ( &LocalPlannerNode::callbackConfigLocalPlanner, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void LocalPlannerNode::callbackConfigLocalPlanner ( mr_local_planner::LocalPlannerConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigLocalPlanner!" );
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void LocalPlannerNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {

    /**
    * @ToDo Wanderer
    * @see http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
    * creates a callback which fills the measurement_laser_ with the information from the sensor_msgs::LaserScan.
    * do not forget to compute the measurement_laser_[i].end_point
    **/
#if PLANNER_EXERCISE >= 20
#else
    measurement_laser_.range_max() = _laser.range_max;
    measurement_laser_.range_min() = _laser.range_min;
    measurement_laser_.resize (_laser.ranges.size());    
    double x_s_r = 0.22, y_s_r = 0.0, theta_s_r=0.0;
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        double range = _laser.ranges[i];
        double angle = _laser.angle_min + i * _laser.angle_increment;   // better to keep incrementing angle because for small increments it could suffer of approx. errors
        // compute measurement in sensor coordinate system        
        double x_m_s = range * cos(angle);
        double y_m_s = range * sin(angle);
        // compute measurement in robot coordinate system
        double x_m_r = x_s_r + x_m_s * cos(theta_s_r) - y_m_s * sin(theta_s_r);
        double y_m_r = y_s_r + x_m_s * sin(theta_s_r) + y_m_s * cos(theta_s_r);
        measurement_laser_[i].length  = range;
        measurement_laser_[i].angle  = angle;
        measurement_laser_[i].end_point  = Point2D(x_m_r, y_m_r);        
    }
    
#endif
}
/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void LocalPlannerNode::callbackOdometry ( const nav_msgs::Odometry &odom ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( odom.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    odom_.set ( odom.pose.pose.position.x, odom.pose.pose.position.y, a );
    odom_.recompute_cached_cos_sin();
}


/**
 * copies incoming pose messages to the base class
 * @param odom
 **/
void LocalPlannerNode::callbackGoal ( const geometry_msgs::Pose2D& goal ) {
    goal_.set ( goal.x, goal.y, goal.theta );
    goal_.recompute_cached_cos_sin();

    start_ = odom_;
    action_state_ = ActionState::INIT;

    Point2D goal_local;
    ROS_INFO ( "goal received! %4.3f,%4.3f",  goal_.x(), goal_.y() );
}

/**
 * copy global path poses into path_ variable
 * @param path_msg
 */
void LocalPlannerNode::callbackPath(const nav_msgs::Path & path_msg) {
    path_.clear();
    for (geometry_msgs::PoseStamped pose : path_msg.poses){
        double yaw = tf::getYaw(pose.pose.orientation);
        Pose2D p(pose.pose.position.x, pose.pose.position.y, yaw);
        path_.push_back(p);
    }
    action_state_ = ActionState::TRACK;
    ROS_INFO_STREAM("Received path of length " << path_.size() << ", mode: " << action_state_);
}

/**
 * Publishes motion commands for a robot
 **/
void LocalPlannerNode::publishMotion () {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish ( cmd );
}

void LocalPlannerNode::visualization(){
    if (path_.size() > 0){
        std::tuple<double, double, double> color = std::make_tuple(0.0, 1.0, 0.0);
        visualizePoint(targetWaypoint_.position(), pub_waypoint_, color);
    }
}


/**
 * Publishes marker of a point
 * @param point point in world coordinates
 * @param pub_point ros publisher
 * @param rgb_color color of the marker
 */
void LocalPlannerNode::visualizePoint(Point2D point, ros::Publisher pub_point, std::tuple<double, double, double> rgb_color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point.x();
    marker.pose.position.y = point.y();
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = std::get<0>(rgb_color);
    marker.color.g = std::get<1>(rgb_color);
    marker.color.b = std::get<2>(rgb_color);
    pub_point.publish(marker);
}
