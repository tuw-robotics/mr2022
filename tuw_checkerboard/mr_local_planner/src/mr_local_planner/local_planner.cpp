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

    /**
     * @ToDo Wanderer
     * change the Maxima Musterfrau to your name
     **/
    cv::putText ( figure_local_.background(), "Alexander Allacher",
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
    //figure_local_.symbol ( Point2D ( 1,2 ), Figure::red ); /// Demo remove afterwards
    //figure_local_.circle ( Point2D ( 2,2 ), 2, Figure::green, 2 ); /// Demo remove afterwards
    // for in measurement_laser_ {}
    for ( int i = 0; i < measurement_laser_.size(); i++ ) {
        //figure_local_.symbol ( measurement_laser_[i].end_point, Figure::red ); /// Demo remove afterwards
        figure_local_.circle ( measurement_laser_[i].end_point, 1, Figure::red, 1 ); /// Demo remove afterwards
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
    
    if ( measurement_laser_.empty() ) {
        cmd_.set ( 0.0, 0.1 );
        return;
    }
    
    double min_v = 0.2;
    double max_v = 0.8;
    double max_r = 0.5;
    double v = 0.0, w = 0.0;
    
    bool near_collision_ahead = false;
    bool collision_ahead = false;
    bool collisio = false;
    
    double sum_left_front = 0.0;
    double sum_left_back = 0.0;
    double min_left_front = 5;
    
    double sum_right_front = 0.0;
    double sum_right_back = 0.0;
    double min_right_front = 5;
    
    double dist_front = 5.0;
    double dist_front_left = 5.0;
    double dist_front_right = 5.0;
    
    for(int i = 0; i < measurement_laser_.size(); i++){
        double x = measurement_laser_[i].end_point.x();
        double y = measurement_laser_[i].end_point.y();
        double dist = measurement_laser_[i].length;
        bool is_left = y > 0;
        bool is_right = !is_left;
        
        // check if more free space is left or right
        if(x > -1 && is_left){
            sum_left_front += dist;
            min_left_front = min(min_left_front, dist);
        }
        else if(x > -1 && is_right){
            sum_right_front += dist;
            min_right_front = min(min_right_front, dist);
        }
        if(x < -1 && is_left){
            sum_left_back += dist;
        }
        else if(x < -1 && is_right){
            sum_right_back += dist;
        }
        
        // points in front of robot
        if(x > 0 && y < 0.3 && y > -0.3){
            // detect collison
            if(x < 0.25) collisio = true;
            if(x < 1) collision_ahead = true;
        }
        else if(x > 0 && y < 0.6 && y > -0.6){
            // detect collison
            if(x < 2.4) near_collision_ahead = true;
            
            // get nearest point in front of robot
            dist_front = min(dist_front, x);
            
            if(is_left) dist_front_left = min(dist_front_left, x);
            if(is_right) dist_front_right = min(dist_front_right, x);
        }
        
        
    }
    
    if(collision_ahead){
        v = 0;
        
        if(abs(dist_front_left-dist_front_right) > 0.5){
            if(dist_front_left>dist_front_right) w = 0.5;
            else w = -0.5;
        }
        else if(abs(sum_left_front - sum_right_front) > 40){
            if(sum_left_front>sum_right_front) w = 0.5;
            else w = -0.5;
        }
        else if (abs(sum_left_back-sum_right_back) > 10) {
            if(sum_left_back>sum_right_back) w = 0.5;
            else w = -0.5;
        }
        
        else w = -0.5;
    }
    else if (near_collision_ahead){
        double speed_factor = dist_front/5;
        v = 0.4 * speed_factor + 0.2;
        if(sum_left_front>sum_right_front) w = 0.5;
        else w = -0.5;
    }
    else if(min_left_front < 0.8)
    {
        double speed_factor = min_left_front/4;
        v = (max_v - min_v) * speed_factor + min_v;
        
        w = -0.5;
    }
    else if(min_right_front < 0.8)
    {
        double speed_factor = min_right_front/4;
        v = (max_v - min_v) * speed_factor + min_v;
        
        w = 0.5;
    }
    else{
        // get slower when objects come close
        double speed_factor = dist_front/4;
        v = (max_v - min_v) * speed_factor + min_v;
        
        double rotation_factor_space = (sum_left_front - sum_right_front) / max(sum_left_front, sum_right_front);
        w = max_r * rotation_factor_space;
    }
    
    if (collisio){
        cmd_.set ( -0.2, 0.1 );
    }
    
    if(v < 0.1) v = 0;
    else v = min(max_v, max(min_v, v));
    
    w = min(max_r, max(-max_r, w));
    
    cmd_.set(v, w);
    
}
void LocalPlanner::wanderer2() {
    
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
