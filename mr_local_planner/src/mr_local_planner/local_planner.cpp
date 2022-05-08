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
    cv::putText ( figure_local_.background(), "Team Red",
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
    for(int i = 0; i < measurement_laser_.size(); ++i) {
        figure_local_.circle(measurement_laser_[i].end_point, 2, Figure::red);
    }
    // for in measurement_laser_ {}
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

#define WANDERER_MIN_DISTANCE (2.0)
#define WANDERER_SPEED_REGULAR (0.2)
#define WANDERER_SPEED_AVOID (0.2)
#define WANDERER_ANGULAR_AVOID (0.5)

void LocalPlanner::wanderer1() {
    /**
    * @ToDo Wanderer
    * write one or two wanderer behaviors to keep the robot at least 120sec driving without a crash by exploring as much as possible.
    * I know it sounds weird but don't get too fancy.
    **/
    double v = 0.0, w = 0.0;
    if (measurement_laser_.empty()) {
        v = 0.0, w = 0.0;
    } else {

        // check a cone in front of the laser to see if it sees a clear path still.

        uint coneOffset = (uint)(measurement_laser_.size() / 16);
        uint angleFront = (uint)(measurement_laser_.size() / 2);
        bool stop = false;
        for (int i = angleFront - coneOffset; i < angleFront + coneOffset; i++) {
            if(measurement_laser_[i].length < WANDERER_MIN_DISTANCE) {
                stop = true;
                break;
            }
        }
        w = stop ? WANDERER_ANGULAR_AVOID : 0.0;
        v = stop ? WANDERER_SPEED_AVOID : WANDERER_SPEED_REGULAR;
    }
    cmd_.set(v,w);
}
void LocalPlanner::wanderer2() {
    /**
    * @ToDo Wanderer
    * OPTIONAL: if you like you can write another one :-)
    **/
    double v = 0.0, w = 0.0;
    if (measurement_laser_.empty()) {
        v = 0.0, w = -0.1;
    } else {

        // check a cone in front of the laser to see if it sees a clear path still.
        // equivalent to wanderer1.

        uint coneOffset = (uint)(measurement_laser_.size() / 16);
        uint angleFront = (uint)(measurement_laser_.size() / 2);
        bool stop = false;
        for (int i = angleFront - coneOffset; i < angleFront + coneOffset; i++) {
            if(measurement_laser_[i].length < WANDERER_MIN_DISTANCE) {
                stop = true;
                break;
            }
        }
        double maxLenMeasureRight = 0.0;
        double maxLenMeasureLeft = 0.0;

        // try to figure out wether the longest measured clear path is to the left or to the right
        // of the robot.

        for (int i = angleFront + coneOffset; i < measurement_laser_.size(); i++) {
            if(measurement_laser_[i].length > maxLenMeasureRight)
                maxLenMeasureRight = measurement_laser_[i].length;
            // FIXME: This loop can be aborted if we see a measurement at max distance of the laser array.
        }
        for (int i = angleFront - coneOffset; i > 0; i--) {
            if(measurement_laser_[i].length > maxLenMeasureLeft)
                maxLenMeasureLeft = measurement_laser_[i].length;
            // FIXME: This loop can be aborted if we see a measurement at max distance of the laser array.
        }
        w = stop ? WANDERER_ANGULAR_AVOID : 0.0;
        v = stop ? WANDERER_SPEED_AVOID : WANDERER_SPEED_REGULAR;
        if(stop && (maxLenMeasureLeft > maxLenMeasureRight)) {
            w = -w;
        }
    }
    cmd_.set(v,w);
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
