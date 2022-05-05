#include "mr_goto/goto.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

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
}
void Goto::bug2() {
    /**
    * @ToDo 4.3 Avoid obstacle
    **/

}
