#include "goto_node.h"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <boost/filesystem.hpp>
#include <nav_msgs/Path.h>
#include <memory>

using namespace moro;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "goto");
    ros::NodeHandle n;
    GotoNode goto_node(n);
    goto_node.init();
    ros::Rate rate(1);

    while (ros::ok())
    {

        goto_node.updatePoseEstimate();

        goto_node.findPath();
        // /// localization
        // self_localization.localization();

        // /// publishes the estimated pose
        // self_localization.publishPoseEstimated();
        // /// publishes the particles
        // self_localization.publishParticles();

        // /// plots measurements
        // self_localization.plot();

        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

GotoNode::GotoNode(ros::NodeHandle &n) : n_(n), n_param_("~")
{
    tf_listener_ = std::make_shared<tf::TransformListener>();
    sub_map_ = n.subscribe("map", 1, &GotoNode::callbackMap, this);
    sub_goal_ = n.subscribe("move_base_simple/goal", 1, &GotoNode::callbackGoal, this);
    sub_pose_estimated_ = n.subscribe("pose_estimated", 1, &GotoNode::callbackPoseEstimated, this);
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_path_ = n.advertise<nav_msgs::Path>("nav_path", 1);
}

void GotoNode::init() {
    
}

void GotoNode::callbackMap(const nav_msgs::OccupancyGrid &map)
{
    ROS_INFO("callbackMap!");
    map_ = map;
    astar_map_ = std::make_shared<Map>(map_.data, map_.info, 80);
}

void GotoNode::callbackGoal(const geometry_msgs::PoseStamped &goal)
{
    ROS_INFO("callbackGoal!");
    tf::Quaternion q;
    tf::quaternionMsgToTF(goal.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal_ = Pose2D(goal.pose.position.x, goal.pose.position.y, yaw);
    goal_set_ = true;
    ROS_INFO("(%f,%f,%f)", goal_.get_x(), goal_.get_y(), goal_.get_theta());
}

void GotoNode::callbackPoseEstimated(const geometry_msgs::PoseWithCovarianceStamped &poseEstimated)
{
    ROS_INFO("callbackPoseEstimated!");
    tf::Quaternion q;
    tf::quaternionMsgToTF(poseEstimated.pose.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_estimated_ = Pose2D(poseEstimated.pose.pose.position.x, poseEstimated.pose.pose.position.y, yaw);
    ROS_INFO("(%f,%f,%f)", pose_estimated_.get_x(), pose_estimated_.get_y(), pose_estimated_.get_theta());
}

void GotoNode::findPath() {
    if(goal_set_) {
        std::shared_ptr<micropather::MicroPather> pather = std::make_shared<micropather::MicroPather>(astar_map_.get());
        micropather::MPVector< void* > path;
        float totalCost = 0;
        void* startState = astar_map_->worldToNode(pose_estimated_.get_x(), pose_estimated_.get_y());
        void* endState = astar_map_->worldToNode(goal_.get_x(), goal_.get_y());
        int result = pather->Solve( startState, endState, &path, &totalCost );

        path_.header.stamp.fromBoost(boost::posix_time::second_clock::universal_time());
        path_.header.seq++;
        path_.header.frame_id = "map";
        path_.poses.clear();

        ROS_INFO("Found path to goal! Total cost: %f", totalCost);
        for (int i = 0; i < path.size(); i++) {
            auto xyWorld = astar_map_->nodeToWorld(path[i]);
            ROS_INFO("Step %d in path: (%f,%f)", i, xyWorld.first, xyWorld.second);
            geometry_msgs::PoseStamped pose;
            pose.header.stamp.fromBoost(boost::posix_time::second_clock::universal_time());
            pose.header.frame_id = "map";
            pose.pose.position.x = xyWorld.first;
            pose.pose.position.y = xyWorld.second;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw ( 0 );
            path_.poses.push_back(pose);
        }

        pub_path_.publish(path_);
    }
}

void GotoNode::updatePoseEstimate()
{
    // tf::StampedTransform transform;
    // try
    // {
    //     tf_listener_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    //     double roll = 0, pitch = 0, yaw = 0;
    //     transform.getBasis().getRPY(roll, pitch, yaw);
    //     pose_estimate_ = Pose2D(transform.getOrigin().x(), transform.getOrigin().y(), yaw);
    // }
    // catch (tf::TransformException ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //     ros::Duration(1.0).sleep();
    // }
}
