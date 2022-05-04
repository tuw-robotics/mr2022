#include "mr_local_planner/local_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace moro;

std::map<LocalPlanner::ControlMode, std::string> LocalPlanner::ControlModeName_ = {
    {DEMO, "DEMO"},
    {STOP, "STOP"},
    {WANDERER1, "WANDERER1"},
    {WANDERER2, "WANDERER2"},
    {BUG1, "BUG1"},
    {BUG2, "BUG2"},
    {TANGENSBUG, "TANGENSBUG"}
};

LocalPlanner::LocalPlanner ( const std::string &ns )
    : loop_count_ ( 0 )
    , figure_local_ ( ns + ", Local View" )
    , action_state_ ( ActionState::NA ) {

}

void LocalPlanner::init() {

    figure_local_.init ( config_.map_pix_x, config_.map_pix_y,
                         config_.map_min_x, config_.map_max_x,
                         config_.map_min_y, config_.map_max_y,
                         config_.map_rotation + M_PI/2.0,
                         config_.map_grid_x, config_.map_grid_y );

    cv::putText ( figure_local_.background(), ControlModeName_[ ( ControlMode ) config_.mode],
                  cv::Point ( 5,figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA );

    cv::putText ( figure_local_.background(), "Team Yellow",
                  cv::Point ( figure_local_.view().cols-250, figure_local_.view().rows-10 ),
                  cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA );
}


void LocalPlanner::plot() {
    if ( config_.plot_data ) plotLocal();
    cv::waitKey ( 10 );
}

void LocalPlanner::plotLocal() {
    figure_local_.clear();
    /**
    * @ToDo Wanderer
    * uses Figure::symbol or Figure::circle to plot the laser measurements in a for loop
    * @Hint figure_local_.symbol (Point2D(1,2), Figure::red );
    **/
#if PLANNER_EXERCISE >= 30
#else
    /**
     * @node your code
     **/
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        figure_local_.circle ( measurement_laser_[i].end_point, 2, Figure::green, 2 );
    }
#endif

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
void LocalPlanner::ai() {
    switch ( config_.mode ) {
    case STOP:
        cmd_.set ( 0, 0 );
        break;
    case DEMO:
        demo();
        break;
    case WANDERER1:
        wanderer1();
        break;
    case WANDERER2:
        wanderer2();
        break;
    case BUG1:
        bug1();
        break;
    case BUG2:
        bug2();
        break;
    case TANGENSBUG:
        tangensbug();
        break;
    default:
        cmd_.set ( 0, 0 );
    }
    loop_count_++;
}


/**
* Demo
**/
void LocalPlanner::demo() {
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.0, w = -0.1;
    } else {
        if ( measurement_laser_[measurement_laser_.size() / 4].length < 1.0 ) {
            w = 0.4;
        } else {
            v = 0.4;
        }
    }
    cmd_.set ( v, w );
}

void LocalPlanner::wanderer1() {
    /**
    * @ToDo Wanderer
    * write one or two wanderer behaviors to keep the robot at least 120sec driving without a crash by exploring as much as possible.
    * I know it sounds weird but don't get too fancy.
    **/
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.0, w = -0.1;
    } else {
        int min_i = 0;
        double min_dis = 100;
        // Find closest Point
        for ( int i = 0; i < measurement_laser_.size(); i++ ) {
            if(measurement_laser_[i].length < min_dis && i >measurement_laser_.size()/3  && i <measurement_laser_.size()*2/3 ) {
                min_i = i;
                min_dis = measurement_laser_[min_i].length;
            }
            if(measurement_laser_[i].length < 3) {
                measurement_laser_[i].length=0;
            }
        }
        // Set linear speed according to closest distance
        v = min_dis / measurement_laser_.range_max() * 0.8;
        if(min_dis <0.05) {
            v=0.0;}
        
        // Safety Bubble
        for ( int i = min_i; i < measurement_laser_.size() && i < min_i+3; i++ ) {
            measurement_laser_[i].length=0;
        }
        for ( int i = min_i; i > 0 && i > min_i-3; i-- ) {
            measurement_laser_[i].length=0;
        }
        // Find Gaps
        int gap_start = measurement_laser_.size()/4;
        int gap_end = measurement_laser_.size()/4;
        int best_gap_start = measurement_laser_.size()/4;
        int best_gap_end = measurement_laser_.size()/4;
        for ( int i = measurement_laser_.size()/4; i < measurement_laser_.size()*3/4; i++ ) {
            if(measurement_laser_[i].length < 0.5) {
                gap_end = i-1;
                if((best_gap_end - best_gap_start) < (gap_end - gap_start)) {
                    best_gap_start = gap_start;
                    best_gap_end = gap_end;
                }
                gap_start = i;
            }
        }
        gap_end = measurement_laser_.size()*3/4-1;
        if((best_gap_end - best_gap_start) < (gap_end - gap_start)) {
            best_gap_start = gap_start;
            best_gap_end = gap_end;
        }
        if(measurement_laser_[(best_gap_end+best_gap_start)/2].angle < 0) {
            w = (measurement_laser_[(best_gap_end+best_gap_start)/2].angle / (M_PI*3/4) * 0.5);
        } else {
            w = (measurement_laser_[(best_gap_end+best_gap_start)/2].angle / (M_PI*3/4) * 0.5);
        }
    }
    cmd_.set ( v, w );
    
}
void LocalPlanner::wanderer2() {
    /**
    * @ToDo Wanderer
    * OPTIONAL: if you like you can write another one :-)
    **/
    double v = 0.0, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.0, w = -0.1;
    } else {
        int min_i = 0;
        // Find closest Point
        for ( int i = measurement_laser_.size()/4; i < measurement_laser_.size()*3/4; i++ ) {
            if(measurement_laser_[i].length < measurement_laser_[min_i].length) {
                min_i = i;
            }
        }
        // Set linear speed according to closest distance
        v = measurement_laser_[min_i].length / measurement_laser_.range_max() * 0.8;
        
        // Safety Bubble
        for ( int i = min_i; i < measurement_laser_.size() && i < min_i+3; i++ ) {
            measurement_laser_[i].length=0;
        }
        for ( int i = min_i; i > 0 && i > min_i-3; i-- ) {
            measurement_laser_[i].length=0;
        }
        // Find Gaps
        int gap_start = 0;
        int gap_end = 0;
        int best_gap_start = 0;
        int best_gap_end = 0;
        for ( int i = 0; i < measurement_laser_.size(); i++ ) {
            if(measurement_laser_[i].length < 0.5) {
                gap_end = i-1;
                if((best_gap_end - best_gap_start) < (gap_end - gap_start)) {
                    best_gap_start = gap_start;
                    best_gap_end = gap_end;
                }
                gap_start = i;
            }
        }
        gap_end = measurement_laser_.size()-1;
        if((best_gap_end - best_gap_start) < (gap_end - gap_start)) {
            best_gap_start = gap_start;
            best_gap_end = gap_end;
        }
        if(measurement_laser_[(best_gap_end+best_gap_start)/2].angle < 0) {
            w = (measurement_laser_[(best_gap_end+best_gap_start)/2].angle / (M_PI*3/4) * 0.5);
        } else {
            w = (measurement_laser_[(best_gap_end+best_gap_start)/2].angle / (M_PI*3/4) * 0.5);
        }
    }
    cmd_.set ( v, w );
}


void LocalPlanner::bug1() {
    /**
    * @ToDo Bug1
    * use goal_, start_ and odom_
    **/
}
void LocalPlanner::bug2() {
    /**
    * @ToDo Bug2
    * use goal_, start_ and odom_
    **/

}
void LocalPlanner::tangensbug() {
    /**
    * @ToDo Tangensbug
    * use goal_, start_ and odom_
    **/

}
