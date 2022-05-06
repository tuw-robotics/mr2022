#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>
#include <mr_planner/PlannerConfig.h>

namespace moro {
/**
 * Robot class
 */
class Planner {
public:
    enum ControlMode {
        STRAIGHT = 0,
        STRAIGHT_AVOID = 1,
        PATH = 2,
        STOP = 3
    };

    enum ActionState {
        NA   = 0,
        INIT = 1,    /// init
        FORWARD = 2 
    };
    static std::map<ControlMode, std::string> ControlModeName_; 
    
    
    Planner(const std::string &ns);        /// Constructor
    void init();                         /// initialization
    void ai();                           /// artificial intelligence calls a behavior
    void plot();                         /// plots sensor input
    
protected:

    Command cmd_;  /// output variables  v, w
    unsigned long loop_count_; /// counts the filter cycles
    Pose2D goal_;  /// goal pose in world coordinates
    Pose2D start_; /// start pose in world coordinates
    Pose2D odom_;  /// current pose based on odometrie
    ActionState action_state_;  /// current action state

    MeasurementLaser measurement_laser_;    /// laser measurements

    void straight();
    void straightAvoid();
    void path();

    mr_planner::PlannerConfig config_;
};
}

#endif // PLANNER_H

