#include "mr_local_planner/local_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <mr_geometry/geometry.h>

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
    cv::putText ( figure_local_.background(), "Team Khaki",
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
    // figure_local_.symbol ( Point2D ( 1,2 ), Figure::red ); /// Demo remove afterwards
    // figure_local_.circle ( Point2D ( 2,2 ), 2, Figure::green, 2 ); /// Demo remove afterwards
    // for in measurement_laser_ {}
    for ( size_t i = 0; i < measurement_laser_.size(); i++ ) {
        figure_local_.symbol ( measurement_laser_[i].end_point , Figure::red ); 
        figure_local_.circle ( measurement_laser_[i].end_point , 2, Figure::green, 2 ); 
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
    static int changeDir = 1;
    double v = 0.5, w = 0.0;
    if ( measurement_laser_.empty() ) {
        v = 0.0, w = 0;
    } else {
        for (size_t i = measurement_laser_.size()*0.4; i < measurement_laser_.size()*0.6; i++ ) {
            if(measurement_laser_[i].length < measurement_laser_.range_max()/4){
                v = 0;
                w = changeDir*0.3;
            }
        }
        for (size_t i = 0; i < measurement_laser_.size(); i++ ) {
            if(measurement_laser_[i].length < measurement_laser_.range_max()/8){
                v = 0;
                w = changeDir*0.3;
            }
        }
    }
    if(v > 0) {
        changeDir = changeDir == 1 ? -1 : 1;
    }
    cmd_.set ( v, w );
}
void LocalPlanner::wanderer2() {
    /**
    * @ToDo Wanderer
    * OPTIONAL: if you like you can write another one :-)
    **/
    cmd_.set ( 0.4, 0.4 );
}

void LocalPlanner::calculateCurrentPosefromTF() {

    try {
        tf_listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
        ROS_INFO_STREAM("tfListener: Origin:" << transform_.getOrigin().x() << ", " << transform_.getOrigin().y() << ", Rotation: " 
        << transform_.getRotation().x() << ", " << transform_.getRotation().y() << ", " << transform_.getRotation().z() << ", " << transform_.getRotation().w());
    } catch ( tf::TransformException ex ) {
        ROS_ERROR ( "%s",ex.what() );
        ros::Duration ( 1.0 ).sleep();
    }

    double roll = 0, pitch = 0, yaw = 0;
    transform_.getBasis().getRPY ( roll, pitch, yaw );

    pose_from_tf_ = Pose2D(transform_.getOrigin().x(), transform_.getOrigin().y(), yaw);

}

void LocalPlanner::bug1() {
    /**
    * @ToDo Bug1
    * use goal_, start_ and odom_
    **/

    /// if using estimated form instead of odom
    if(true) {
        calculateCurrentPosefromTF();
        odom_ = pose_from_tf_;
    }
    double v = 0.0, w = 0.0;
 

    if ( action_state_ != ActionState::NA ) {

        double goal_direction = atan2(goal_.y() - odom_.y(), goal_.x() - odom_.x());
        double goal_distance = sqrt(pow(goal_.y() - odom_.y(), 2) + pow(goal_.x() - odom_.x(), 2));
        double current_orientation = odom_.theta();
        double goal_orientation = goal_.theta();

        if ( action_state_ == ActionState::INIT ) {
            ROS_INFO("goal received, start planning;)");
            action_state_ = ActionState::TURN;
        }
        if ( action_state_ == ActionState::TURN ) {
            ROS_DEBUG("Angle difference: %f", angle_difference(goal_direction, current_orientation));
            if ( angle_difference(goal_direction, current_orientation) > 0.2 ) {
                w = 0.2;
            } else if ( angle_difference(goal_direction, current_orientation) < -0.2 ) {
                w = -0.2;
            } else {
                ROS_INFO("goal direction reached, start driving");
                action_state_ = ActionState::STRAIGHT;
            }
        }
        if ( action_state_ == ActionState::STRAIGHT ) {
            if ( angle_difference(goal_direction, current_orientation) > 0.2 ) {
                w = 0.2;
            } else if ( angle_difference(goal_direction, current_orientation) < -0.2 ) {
                w = -0.2;
            } else {
                w = 0;
            }
            if (goal_distance > 0.2) {
                v = 0.2;
            } else {
                ROS_INFO("goal reached, start turning");
                action_state_ = ActionState::FINAL_ORIENTATION;
            }
        }
        if ( action_state_ == ActionState::FINAL_ORIENTATION ) {
            ROS_DEBUG("Angle difference: %f", angle_difference(goal_orientation, current_orientation));
            if ( angle_difference(goal_orientation, current_orientation) > 0.2 ) {
                w = 0.2;
            } else if ( angle_difference(goal_orientation, current_orientation) < -0.2 ) {
                w = -0.2;
            } else {
                ROS_INFO("goal orientation reached, goal reached");
                action_state_ = ActionState::NA;
            }
        }

    }

    cmd_.set ( v, w );
    
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
