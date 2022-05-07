#include "goto_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>

using namespace moro;
int main ( int argc, char **argv ) {

    ros::init ( argc, argv, "goto" );  /// initializes the ros node with default name
    ros::NodeHandle n;
    GotoNode planner ( n );
    planner.init();
    ros::Rate rate ( 10 );  /// ros loop frequency synchronized with the wall time (simulated time)

    while ( ros::ok() ) {
        // reads predicted pose
        planner.predictedPoseListener();

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
GotoNode::GotoNode ( ros::NodeHandle & n )
    : Goto ( ros::NodeHandle ( "~" ).getNamespace() ),
      n_ ( n ),
      n_param_ ( "~" ) {

     sub_laser_ = n.subscribe("scan", 1000, &GotoNode::callbackLaser, this);


    /**
     * @ToDo GoTo
     * subscribes the fnc callbackGoal to goal and fnc callbackOdometry to odom
     **/
#if PLANNER_EXERCISE >= 10
#else
    /**
     * @node your code
     **/
    // sub_goal_ =
     sub_goal_ = n.subscribe("move_base_simple/goal", 1000, &GotoNode::callbackGoal, this);
     sub_odom_ = n.subscribe("odom", 1000, &GotoNode::callbackOdometry, this);
#endif
    tf_listener_ = std::make_shared<tf::TransformListener>();

    /// defines a publisher for velocity commands
    pub_cmd_ = n.advertise<geometry_msgs::Twist> ( "cmd_vel", 1 );

    reconfigureFnc_ = boost::bind ( &GotoNode::callbackConfigGoto, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}

void GotoNode::callbackConfigGoto ( mr_goto::GotoConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigGoto!" );
    config_ = config;
    init();
}

/**
 * copies incoming laser messages to the base class
 * @param laser
 **/
void GotoNode::callbackLaser ( const sensor_msgs::LaserScan &_laser ) {

    measurement_laser_.range_max() = _laser.range_max;
    measurement_laser_.range_min() = _laser.range_min;
    measurement_laser_.resize ( _laser.ranges.size() );
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
     measurement_laser_[i].length  = _laser.ranges[i];
     measurement_laser_[i].angle  = _laser.angle_min +_laser.angle_increment*i;
     measurement_laser_[i].end_point  = Point2D(0.22+measurement_laser_[i].length * cos(measurement_laser_[i].angle), measurement_laser_[i].length * sin(measurement_laser_[i].angle));
    }
}
/**
 * copies incoming odometry messages to the base class
 * @param odom
 **/
void GotoNode::callbackOdometry ( const nav_msgs::Odometry &odom ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( odom.pose.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    double a = yaw;
    odom_.set ( odom.pose.pose.position.x, odom.pose.pose.position.y, a );
    odom_.recompute_cached_cos_sin();
    //ROS_INFO ( "callbackOdomGoto!" );
    //ROS_INFO ( "odom received! %4.3f,%4.3f and %3.2f rad",  odom_.x(), odom_.y(), odom_.theta() );
}


/**
 * copies incoming pose messages to the base class
 * @param odom
 **/
void GotoNode::callbackGoal ( const geometry_msgs::PoseStamped& goal ) {
    tf::Quaternion q;
    tf::quaternionMsgToTF ( goal.pose.orientation, q );
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3 ( q ).getRPY ( roll, pitch, yaw );
    goal_.set ( goal.pose.position.x, goal.pose.position.y, yaw );
    goal_.recompute_cached_cos_sin();

    start_ = odom_;
    goal_set_ = true;
    action_state_ = ActionState::INIT;

    Point2D goal_local;
    ROS_INFO ( "goal received! %4.3f,%4.3f and %3.2f rad",  goal_.x(), goal_.y(), goal_.theta() );
}

void GotoNode::predictedPoseListener () {
    tf::StampedTransform transform;
    try {
        tf_listener_->lookupTransform("/map", "/pose_estimated_tf", ros::Time(0), transform);
        double roll = 0, pitch = 0, yaw = 0;
        transform.getBasis().getRPY ( roll, pitch, yaw );
        pred_pose_ = Pose2D ( transform.getOrigin().x(),  transform.getOrigin().y(), yaw );
        //ROS_INFO ( "pose received! %4.3f,%4.3f and %3.2f rad",  pred_pose_.x(), pred_pose_.y(), pred_pose_.theta() );
    } catch ( tf::TransformException ex ) {
    }
}


/**
 * Publishes motion commands for a robot
 **/
void GotoNode::publishMotion () {
    geometry_msgs::Twist cmd;
    /// creates motion command
    cmd.linear.x = cmd_.v();
    cmd.linear.y = 0.;
    cmd.angular.z = cmd_.w();
    /// publishes motion command
    pub_cmd_.publish ( cmd );
}
