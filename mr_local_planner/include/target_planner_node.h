#ifndef TARGET_PLANNER_NODE_H
#define TARGET_PLANNER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <mr_target_planner/target_planner.h>
#include <tf2_ros/transform_listener.h>

/**
 * class to cover the ros communication
 **/
class TargetPlannerNode  : public moro::TargetPlanner {
  public:
    TargetPlannerNode(ros::NodeHandle &n); /// Constructor
    void publishMotion();                  /// publishes the motion commands
    void callback_goal(const geometry_msgs::PoseStamped &); /// callback function to execute on incoming goal commands

  private:
      ros::NodeHandle n_;                                                                              /// node handler to the root node
    ros::NodeHandle n_param_;                                                                        /// node handler to the current node
    ros::Subscriber sub_goal_; /// Subscriber to the goal in world coordinates
    ros::Publisher pub_cmd_;   /// publisher for the motion commands
};


#endif // PLANNER_LOCAL_NODE_H