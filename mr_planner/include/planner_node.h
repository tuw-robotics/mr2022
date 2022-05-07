#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <mr_planner/planner.h>
#include <mr_planner/PlannerConfig.h>
/**
 * class to cover the ros communication
 **/
class PlannerNode : public moro::Planner {
public:
    PlannerNode ( ros::NodeHandle & n ); /// Constructor
    void publishMotion ();      /// publishes the motion commands 
    moro::Pose2D estimatedPose();
    void updateEstimatedPose();

private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
    ros::Subscriber sub_goal_;  /// Subscriber to the goal in world coordinates
    ros::Publisher pub_cmd_;    /// publisher for the motion commands
    tf2_ros::Buffer tf_buffer_;   /// Buffer for tf2 messages
    tf2_ros::TransformListener tf_listener_;  /// Listener for tf2 messages
    void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
    void callbackGoal ( const geometry_msgs::PoseStamped& );   /// callback function to execute on incoming goal commands
    void callbackConfigPlanner ( mr_planner::PlannerConfig &config, uint32_t level ); /// callback function on incoming parameter changes
    dynamic_reconfigure::Server<mr_planner::PlannerConfig> reconfigureServer_; /// parameter server stuff
    dynamic_reconfigure::Server<mr_planner::PlannerConfig>::CallbackType reconfigureFnc_;  /// parameter server stuff
    moro::Pose2D pose_estimation_;
};

#endif // PLANNER_LOCAL_NODE_H
