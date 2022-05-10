#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <mr_geometry/geometry.h>

namespace moro {
/**
 * Robot class
 */
class CustomPlanner {
public:
    enum ActionState {
        NA   = 0,    /// init
        INIT = 1,    /// init
        WAIT = 2,    /// wait for new commands 
        TURN = 3,     /// turn to goal
        STRAIGHT = 4, 
        WALL_FOLLOW_LEFT = 5, 
        WALL_FOLLOW_RIGHT = 6 
    };
    
    CustomPlanner(const std::string &ns); /// Constructor
    void ai();                           /// artificial intelligence calls a behavior
    
protected:

    Command cmd_;  /// output variables  v, w
    Pose2D goal_;  /// goal pose in world coordinates
    Pose2D start_; /// start pose in world coordinates
    Pose2D odom_;  /// current pose based on odometrie
    ActionState action_state_;  /// current action state
    MeasurementLaser measurement_laser_;    /// laser measurements
};
}

#endif // PLANNER_LOCAL_H

