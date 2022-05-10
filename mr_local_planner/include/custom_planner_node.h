#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <mr_custom_planner/custom_planner.h>

#include <tf/transform_listener.h>

/**
 * class to cover the ros communication
 **/
class CustomPlannerNode : public moro::CustomPlanner {
public:
    CustomPlannerNode ( ros::NodeHandle & n ); /// Constructor
    void publishMotion ();      /// publishes the motion commands 
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
    ros::Subscriber sub_odom_;  /// Subscriber to the odom measurements
    ros::Subscriber sub_goal_;  /// Subscriber to the goal in world coordinates
    ros::Publisher pub_cmd_;    /// publisher for the motion commands
    void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
    void callbackGoal(const geometry_msgs::PoseStamped& goal);   /// callback function to execute on incoming goal commands
    void callbackOdometry ( const nav_msgs::Odometry & ) ;     /// callback function to execute on incoming odometry data

    
    /// declaration of the transform listener and the variable holding the received transform
    tf::TransformListener tf_listener_;
    tf::StampedTransform tf_;
    
    //ros::Subscriber sub_movement_goal_;
    //void callbackMovementGoal(const geometry_msgs::Pose2D& ) ;
    
};

#endif // PLANNER_LOCAL_NODE_H
