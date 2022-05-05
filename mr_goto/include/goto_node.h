#ifndef GOTO_NODE_H
#define GOTO_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <mr_goto/goto.h>
#include <../../opt/ros/noetic/include/geometry_msgs/Point.h>
#include <mr_goto/GotoConfig.h>
/**
 * class to cover the ros communication
 **/
class GotoNode : public moro::Goto {
public:
    GotoNode ( ros::NodeHandle & n ); /// Constructor
    void publishMotion ();      /// publishes the motion commands 
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
    ros::Subscriber sub_odom_;  /// Subscriber to the odom measurements
    ros::Subscriber sub_goal_;  /// Subscriber to the goal in world coordinates
    ros::Publisher pub_cmd_;    /// publisher for the motion commands
    void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
    void callbackGoal ( const geometry_msgs::PoseStamped& );   /// callback function to execute on incoming goal commands
    void callbackOdometry ( const nav_msgs::Odometry & ) ;     /// callback function to execute on incoming odometry data
    void callbackConfigGoto ( mr_goto::GotoConfig &config, uint32_t level ); /// callback function on incoming parameter changes
    dynamic_reconfigure::Server<mr_goto::GotoConfig> reconfigureServer_; /// parameter server stuff
    dynamic_reconfigure::Server<mr_goto::GotoConfig>::CallbackType reconfigureFnc_;  /// parameter server stuff
};

#endif // GOTO_NODE_H
