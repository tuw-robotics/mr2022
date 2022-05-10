#include "mr_custom_planner/custom_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace moro;

CustomPlanner::CustomPlanner ( const std::string &ns )
    : action_state_ ( ActionState::NA ),
      prev_goal_ (Pose2D (0,0,0)),
      wait_count_ ( 0 ),
      waiting_ ( false ) {

}

/// Implements AI as in the local_planner variant, features only the GoTo "bug1" mode
void CustomPlanner::ai() {
    double v = 0.0, w = 0.0;
    if (measurement_laser_.empty() || (goal_.x()==0 && goal_.y()==0)) {
        v = 0.0, w = 0.0;
    } else {

        cv::Vec2d robot_pointing(cos(odom_.theta()), sin(odom_.theta()));
        cv::Vec2d odom2goal(goal_.x()-odom_.x(), goal_.y()-odom_.y());

        double dot = robot_pointing(0) * odom2goal(0) + robot_pointing(1) * odom2goal(1);
        double det = robot_pointing(0) * odom2goal(1) - robot_pointing(1) * odom2goal(0);
        double angle = atan2(det, dot);

        //target angle for the robot to turn to
        double target_theta = odom_.theta()+angle;

        //check if a new goal has been set, reset driving if yes
        if(prev_goal_.x()!=goal_.x() && prev_goal_.y()!=goal_.y()){
            waiting_ = false;
            wait_count_ = 0;
        }
        prev_goal_ = goal_;

        //check if rotation has been completed and if the timer to counteract oversteering has run down
        if(waiting_ && wait_count_==10){
            double dist = sqrt(pow(odom2goal(0),2)+pow(odom2goal(1),2));
            v = 1.0;
            if(dist<1){
               v = 0.0;
            }
        }else if(waiting_){
            wait_count_++;
        }else{
            //if the rotation is within a tolerance, stop turning
            if(abs(target_theta-odom_.theta())<0.5){
                waiting_ = true;
            }else{
                if(target_theta>odom_.theta())
                    w = 1.0;
                else if(target_theta<odom_.theta())
                    w = -1.0;
            }
        }
    }

    cmd_.set(v,w);
}