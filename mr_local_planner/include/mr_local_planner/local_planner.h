#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <mr_local_planner/LocalPlannerConfig.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/transform_datatypes.h>
#include <random>

namespace moro {
/**
 * Robot class
 */
class LocalPlanner {
public:
    enum ControlMode {
        STOP = 0,
        DEMO = 1,
        WANDERER1 = 2,
        WANDERER2 = 3,
        BUG1 = 4,
        BUG2 = 5,
        TANGENSBUG = 6,
        GOTO = 7
    };
    enum ActionState {
        NA   = 0,    /// init
        INIT = 1,    /// init
        WAIT = 2,    /// wait for new commands 
        TURN = 3,     /// turn to goal
        STRAIGHT = 4, 
        WALL_FOLLOW_LEFT = 5, 
        WALL_FOLLOW_RIGHT = 6,
        /// @ToDo for goto expand action state if needed
        TURN_LEFT = 7,  // used in wanderer1
        TURN_RIGHT = 8, // used in wanderer2
        /// Custom action-states for goto
        TRACK = 9,
        ALIGN = 10,
        REPLAN = 11
    };
    
    static std::map<ControlMode, std::string> ControlModeName_; 
    
    
    LocalPlanner(const std::string &ns); /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behavior
    void plot();                         /// plots sensor input
    void updateTransform(geometry_msgs::TransformStamped& new_transform, geometry_msgs::TransformStamped& new_laser_transform);

protected:
    std::random_device rd{};
    std::mt19937 gen{rd()};

    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles
    Pose2D goal_;  /// goal pose in world coordinates
    Pose2D start_; /// start pose in world coordinates
    Pose2D odom_;  /// current pose based on odometrie
    Pose2D transform_; /// current transformation published by self localization using the tf tree
    Pose2D laser_transform_; /// current transformation of lidar published by self localization using the tf tree
    std::vector<Pose2D> path_;  /// path as sequence of points
    Pose2D targetWaypoint_;   /// goal waypoint (pure pursuit) in world coordinates
    ActionState action_state_;  /// current action state

    MeasurementLaser measurement_laser_;    /// laser measurements

    Figure figure_local_;  /// Figure for data visualization

    void demo();            /// Demo behavior
    void wanderer1();       /// Wanderer behavior
    void wanderer2();       /// Wanderer behavior
    void bug1();            /// Bug1 behavior
    void bug2();            /// Bug2 behavior
    void tangensbug();      /// Tangensbug behavior
    void goto_plan();            /// goto local plan
    std::tuple<double, double> path_tracking(Pose2D target, double lookahead);   /// geometric path tracking algorithm
    std::tuple<double, double> alternative_planner(Pose2D target);
    std::tuple<double, double> final_alignment(); /// perform final rotation to align robot orientation
    void plotLocal();       /// plots sensor input in robot coordinates

    bool nextWaypointReachable(Pose2D targetWaypoint);
    
    // Wanderer
    ros::Time rotationStart = ros::Time::now();   // when a in-place rotation started
    double rotationDuration = 0.0;          // how long the rotation should last

    int getNextWaypointID(const double lookahead);  // compute the next waypoint along the path

    mr_local_planner::LocalPlannerConfig config_;
};
}

#endif // PLANNER_LOCAL_H

