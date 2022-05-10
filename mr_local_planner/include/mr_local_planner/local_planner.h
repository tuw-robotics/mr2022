#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <mr_local_planner/LocalPlannerConfig.h>

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
        TANGENSBUG = 6
    };
    enum ActionState {
        NA   = 0,    /// init
        INIT = 1,    /// init
        WAIT = 2,    /// wait for new commands 
        TURN = 3,     /// turn to goal
        STRAIGHT = 4, 
        WALL_FOLLOW_LEFT = 5, 
        WALL_FOLLOW_RIGHT = 6 
        /// @ToDo for goto expand action state if needed
    };
    static std::map<ControlMode, std::string> ControlModeName_; 
    
    
    LocalPlanner(const std::string &ns); /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behavior
    void plot();                         /// plots sensor input
    
protected:
    
    //wait counter variables to counteract oversteering
    int wait_count_;
    bool waiting_;
    
    //variable to store the previous goal position
    Pose2D prev_goal_;
    
    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles
    Pose2D goal_;  /// goal pose in world coordinates
    Pose2D start_; /// start pose in world coordinates
    Pose2D odom_;  /// current pose based on odometrie
    ActionState action_state_;  /// current action state

    MeasurementLaser measurement_laser_;    /// laser measurements

    Figure figure_local_;  /// Figure for data visualization

    void demo();            /// Demo behavior
    void wanderer1();       /// Wanderer behavior
    void wanderer2();       /// Wanderer behavior
    void bug1();            /// Bug1 behavior
    void bug2();            /// Bug2 behavior
    void tangensbug();      /// Tangensbug behavior
    void plotLocal();       /// plots sensor input in robot coordinates
    mr_local_planner::LocalPlannerConfig config_;
};
}

#endif // PLANNER_LOCAL_H

