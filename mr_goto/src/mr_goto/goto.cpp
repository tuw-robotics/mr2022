#include "mr_goto/goto.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>

using namespace cv;
using namespace moro;

std::map<Goto::ControlMode, std::string> Goto::ControlModeName_ = {
    {STOP, "STOP"},
    {BUG1, "BUG1"},
    {BUG2, "BUG2"},
};

Goto::Goto ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Local View" )
    , action_state_ ( ActionState::NA ) {

}

void Goto::init() {

    figure_local_.init ( config_.map_pix_x, config_.map_pix_y,
                         config_.map_min_x, config_.map_max_x,
                         config_.map_min_y, config_.map_max_y,
                         config_.map_rotation + M_PI/2.0,
                         config_.map_grid_x, config_.map_grid_y );

    cv::putText ( figure_local_.background(), ControlModeName_[ ( ControlMode ) config_.mode],
                  cv::Point ( 5,figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA );

    cv::putText ( figure_local_.background(), "Team yellow",
                  cv::Point ( figure_local_.view().cols-250, figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA );
}


void Goto::plot() {
    if ( config_.plot_data ) plotLocal();
    cv::waitKey ( 10 );
}

void Goto::plotLocal() {
    figure_local_.clear();
    
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        figure_local_.circle ( measurement_laser_[i].end_point, 2, Figure::green, 2 );
    }

    /**
    * @ToDo GoTo
    * The tf of the odom node is used to visualize the goal and start.
    * It has to be inverted because we want to visualize in world space.
    */
#if PLANNER_EXERCISE >= 31
#else
    /**
     * @node your code
     **/
#endif
    cv::imshow ( figure_local_.title(),figure_local_.view() );
}

/**
 * @note these are the different robot behaviours.
 * At least one implementation is required for Exercise 2 Part 2
 */
void Goto::ai() {
    switch ( config_.mode ) {
    case STOP:
        cmd_.set ( 0, 0 );
        break;
    case BUG1:
        bug1();
        break;
    case BUG2:
        bug2();
        break;
    default:
        cmd_.set ( 0, 0 );
    }
    loop_count_++;
}


void Goto::bug1() {
    /**
    * @ToDo 4.2 Simple, no Obstacle
    **/
    if(goal_set_ && (!config_.use_path || path_set_)) {
        Pose2D target;

        if(config_.use_path) {
            target = path_next_step_;
        } else {
            target = goal_;
        }

        double v = 0.0, w = 0.0;
        double angle = 0;         // angle to goal
        double dx = target.x() - pred_pose_.x();
        double dy = target.y() - pred_pose_.y();
        double dist = sqrt(pow(dx,2)+pow(dy,2));    // distance to goal
        double angle_diff;      // difference between target angle and robot angle
        angle = atan2(dy, dx);
        angle_diff = moro::angle_difference(angle, pred_pose_.theta());
        // goal not reached: turn towards the goal and drive there
        if(dist > 0.2) {
            w = angle_diff/M_PI_4 * 0.5;
            // only drive forward if looking towards the goal
            if(abs(angle_diff) < 0.5) {
                v = (1 - abs(angle_diff)/M_PI) * 0.8 * (min(dist, 3.0)/4 + 0.25);
            }
        } 
        else if (target.equal(goal_, 0.0001)) {
            // goal reached, correct orientation
            if(abs(moro::angle_difference(target.theta(), pred_pose_.theta())) > 0.1) {
                if( moro::angle_difference(target.theta(), pred_pose_.theta()) > 0) {
                    w = 0.15;
                } else {
                    w = -0.15;
                }
            } 
            // goal pose reached
            else {
                goal_set_ = false;
            }
        }
        // cap w
        w = min(w,0.5);
        w = max(w,-0.5);
        cmd_.set ( v, w );
    }
        
}
void Goto::bug2() {
    /**
    * @ToDo 4.3 Avoid obstacle
    **/

}
