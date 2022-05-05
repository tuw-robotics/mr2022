#ifndef MR_NODE_H
#define MR_NODE_H

#include <memory>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <mr_geometry/geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include "mr_goto/map.h"
#include "mr_goto/micropather.h"
#include "nav_msgs/Path.h"

namespace moro
{
    /**
 * class to cover the ros communication for the self-localization
 **/
    class GotoNode
    {
    public:
        GotoNode(ros::NodeHandle &n); /// Constructor
        void init();                  /// initialization
        void updatePoseEstimate();
        void findPath();

    private:
        ros::NodeHandle n_;       /// node handler to the root node
        ros::NodeHandle n_param_; /// node handler to the current node

        ros::Subscriber sub_map_;  /// Subscriber to receive the published map as an occupancy grid
        ros::Subscriber sub_goal_; /// Subscriber to the goal in world coordinates
        ros::Subscriber sub_pose_estimated_; /// Subscriber to the goal in world coordinates
        ros::Publisher pub_path_;   /// publisher for the planned path to the goal
        ros::Publisher pub_cmd_;   /// publisher for the motion commands

        std::shared_ptr<tf::TransformListener> tf_listener_; /// listener to receive transformation messages -> to get the laser pose

        nav_msgs::OccupancyGrid map_;
        std::shared_ptr<Map> astar_map_;
        Pose2D goal_;
        bool goal_set_ = false;
        Pose2D pose_estimated_;
        nav_msgs::Path path_;

        void callbackMap(const nav_msgs::OccupancyGrid &);     /// callback function to catch the map
        void callbackGoal(const geometry_msgs::PoseStamped &); /// callback function to catch goal updates
        void callbackPoseEstimated(const geometry_msgs::PoseWithCovarianceStamped &); /// callback function to catch goal updates
    };
}

#endif // MR_NOTE_H
