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

    while ( ros::ok() ) {

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
    sub_laser_ = n.subscribe ( "scan" , 1, &LocalPlannerNode::callbackLaser, this);
#endif


    /**
     * @Done6 GoTo
     * subscribes the fnc callbackGoal to goal and fnc callbackOdometry to odom
     **/
#if PLANNER_EXERCISE >= 10
#else
    /**
     * @node your code
     **/
    sub_goal_ = n_.subscribe ( "move_base_simple/goal", 1, &LocalPlannerNode::callbackGoal, this );
    sub_odom_ = n_.subscribe ( "odom", 1, &LocalPlannerNode::callbackOdometry, this );
#endif

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

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
    /**
     * @node your code
     **/
    measurement_laser_.range_max() = _laser.range_max;  /// @ToDo
    measurement_laser_.range_min() = _laser.range_min; /// @ToDo
    measurement_laser_.resize ( _laser.ranges.size() ); /// @ToDo
    for ( size_t i = 0; i < measurement_laser_.size(); i++ ) {
        measurement_laser_ [i].length = _laser.ranges [i];
        measurement_laser_ [i].angle = _laser.angle_min + ( _laser.angle_increment * i );
        measurement_laser_ [i].end_point = Point2D (
            cos ( measurement_laser_ [i].angle ) * measurement_laser_ [i].length + 0.22,
            sin ( measurement_laser_ [i].angle ) * measurement_laser_ [i].length
        );
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
    //ROS_INFO ( "odom received! %4.3f,%4.3f",  odom_.x(), odom_.y() );
}


/**
 * copies incoming pose messages to the base class
 * @param odom
 **/
void LocalPlannerNode::callbackGoal ( const geometry_msgs::PoseStamped& goal ) {
    ROS_INFO ( "goal received! %4.3f,%4.3f",  goal.pose.position.x, goal.pose.position.y );
     tf::Quaternion q;
    tf::quaternionMsgToTF ( goal.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    goal_.set(goal.pose.position.x, goal.pose.position.y, a);    
    
    //goal_.set ( goal.x, goal.y, goal.theta );
    goal_.recompute_cached_cos_sin();

    start_ = odom_;
    action_state_ = ActionState::INIT;

    Point2D goal_local;
    //ROS_INFO ( "goal received! %4.3f,%4.3f",  goal_.x(), goal_.y() );
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
