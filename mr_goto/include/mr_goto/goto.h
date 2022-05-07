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
    bool goal_set_ = false;
    ActionState action_state_;  /// current action state

    MeasurementLaser measurement_laser_;    /// laser measurements

    Figure figure_local_;  /// Figure for data visualization

    void bug1();            /// Bug1 behavior
    void bug2();            /// Bug2 behavior
    void plotLocal();       /// plots sensor input in robot coordinates
    mr_goto::GotoConfig config_;
};
}

#endif // GOTO_H

