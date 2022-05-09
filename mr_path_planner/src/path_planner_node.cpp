#include "path_planner_node.h"
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

    ros::init(argc, argv, "path_planner");
    ros::NodeHandle n;
    PathPlannerNode path_planner(n);
    path_planner.init();
    ros::Rate rate(10);

    while (ros::ok())
    {

        path_planner.updatePoseEstimate();

        path_planner.findPath();
        
        /// calls all callbacks waiting in the queue
        ros::spinOnce();

        /// sleep for the time remaining to let us hit our publish rate
        rate.sleep();
    }
    return 0;
}

PathPlannerNode::PathPlannerNode(ros::NodeHandle &n) : n_(n), n_param_("~")
{
    tf_listener_ = std::make_shared<tf::TransformListener>();
    sub_map_ = n.subscribe("map", 1, &PathPlannerNode::callbackMap, this);
    sub_goal_ = n.subscribe("move_base_simple/goal", 1, &PathPlannerNode::callbackGoal, this);
    // sub_pose_estimated_ = n.subscribe("pose_estimated", 1, &PathPlannerNode::callbackPoseEstimated, this);
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_path_ = n.advertise<nav_msgs::Path>("nav_path", 1);

    reconfigureFnc_ = boost::bind ( &PathPlannerNode::callbackConfigPathPlanner, this,  _1, _2 );
    reconfigureServer_.setCallback ( reconfigureFnc_ );
}


void PathPlannerNode::callbackConfigPathPlanner ( mr_path_planner::PathPlannerConfig &config, uint32_t level ) {
    ROS_INFO ( "callbackConfigPathPlanner!" );
    config_ = config;
    init();
}

void PathPlannerNode::init() {
    if(astar_map_ != nullptr) {
        initMapAndPather();
    }
}

void PathPlannerNode::callbackMap(const nav_msgs::OccupancyGrid &map)
{
    if (map_.data != map.data || map_.info.height != map.info.height ||
        map_.info.width != map.info.width || map_.info.resolution != map.info.resolution ||
        map_.info.origin != map.info.origin) {
            ROS_INFO("callbackMap!");
            map_ = map;
            initMapAndPather();
        }
}

void PathPlannerNode::initMapAndPather() {
    MapOptions options_ = {
        config_.map_scale, // scale
        static_cast<uint8_t>(config_.map_blur_iter), // blur iterations
        static_cast<uint8_t>(config_.map_blur_size), // blur filter size
        config_.diagonal_move, // diagonal paths
        static_cast<uint8_t>(config_.occupancy_thres) // occupancy threshold (0-255)
    };
    astar_map_ = std::make_shared<Map>(map_, options_);
    micropather_ = std::make_shared<micropather::MicroPather>(astar_map_.get());
}

void PathPlannerNode::callbackGoal(const geometry_msgs::PoseStamped &goal)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(goal.pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    goal_ = Pose2D(goal.pose.position.x, goal.pose.position.y, yaw);
    goal_set_ = true;
    ROS_INFO("callbackGoal: (%f,%f,%f)", goal_.get_x(), goal_.get_y(), goal_.get_theta());
}

void PathPlannerNode::findPath() {
    if(goal_set_) {
        double dx = goal_.x() - pose_estimated_.x();
        double dy = goal_.y() - pose_estimated_.y();
        double dist = sqrt(pow(dx,2)+pow(dy,2));    // distance to goal

        if(dist > 0.2) {
            micropather::MPVector< void* > path;
            float totalCost = 0;
            void* startState = astar_map_->worldToNode(pose_estimated_);
            void* endState = astar_map_->worldToNode(goal_);
            int result = micropather_->Solve( startState, endState, &path, &totalCost );

            path_.header.stamp.fromBoost(boost::posix_time::second_clock::universal_time());
            path_.header.seq++;
            path_.header.frame_id = "map";
            path_.poses.clear();

            ROS_INFO("Found path to goal! Total cost: %f", totalCost);
            for (int i = 0; i < path.size(); i++) {
                auto pointWorld = astar_map_->nodeToWorld(path[i]);
                ROS_INFO("Step %d in path: (%f,%f)", i, pointWorld.get_x(), pointWorld.get_y());
                geometry_msgs::PoseStamped pose;
                pose.header.stamp.fromBoost(boost::posix_time::second_clock::universal_time());
                pose.header.frame_id = "map";
                pose.pose.position.x = pointWorld.get_x();
                pose.pose.position.y = pointWorld.get_y();
                pose.pose.position.z = 0;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw ( 0 );
                path_.poses.push_back(pose);
            }

            pub_path_.publish(path_);
        } else {
            goal_set_ = false;
        }        
    }
}

void PathPlannerNode::updatePoseEstimate()
{
    tf::StampedTransform transform;
    try {
        tf_listener_->lookupTransform("/map", "/pose_estimated_tf", ros::Time(0), transform);
        double roll = 0, pitch = 0, yaw = 0;
        transform.getBasis().getRPY ( roll, pitch, yaw );
        pose_estimated_ = Pose2D ( transform.getOrigin().x(),  transform.getOrigin().y(), yaw );
    } catch ( tf::TransformException ex ) {
    }
}
