#ifndef GOTO_H
#define GOTO_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <mr_goto/GotoConfig.h>

namespace moro {
/**
 * Robot class
 */
class Goto {
public:
    enum ControlMode {
        STOP = 0,
        BUG1 = 1,
        BUG2 = 2
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
    
    
    Goto(const std::string &ns); /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behavior
    void plot();                         /// plots sensor input
    
protected:

    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles
    Pose2D goal_;  /// goal pose in world coordinates
    Pose2D start_; /// start pose in world coordinates
    Pose2D odom_;  /// current pose based on odometrie
    Pose2D pred_pose_;  /// current pose based on self_localization
    Pose2D path_next_step_; /// next point on the path to target
    bool goal_set_ = false;
    bool path_set_ = false;
    ActionState action_state_;  /// current action state

    MeasurementLaser measurement_laser_;    /// laser measurements

    Figure figure_local_;  /// Figure for data visualization

    void bug1();            /// Bug1 behavior
    void bug2();            /// Bug2 behavior
    void plotLocal();       /// plots sensor input in robot coordinates
    mr_goto::GotoConfig config_;
    
    // Members and functions for DWA
    cv::Mat Vs_; /// DWA search space
    Figure figure_search_space_;  /// Figure for command visualization
    boost::posix_time::ptime timestamp_last_update_;  /// time of the last calculated command
    boost::posix_time::time_duration duration_last_update_;  /// time since the previous calculated command
    
    void fill_search_space();
    Command select_from_dw(Pose2D target);
    double NF(double v, double w, Pose2D target);
    bool updateTimestamp(const boost::posix_time::ptime& t);
};
}

#endif // GOTO_H

