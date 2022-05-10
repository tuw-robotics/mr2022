#include "mr_goto/goto.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <tf/transform_datatypes.h>
#include <time.h>

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
    , figure_search_space_ ( ns + ", Command Space" )
    , action_state_ ( ActionState::NA )
    , Vs_( 51, 91, CV_8UC1) {

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
    
    //init command space window
    int size = 400;
    figure_search_space_.setLabel ( "w=%4.2f deg/sec","v=%4.2f m/sec" );
    figure_search_space_.init(size * 2, size, -90.0, 90.0, 5.0, 0.0, M_PI, 1.0, 10.0);
    
    Vs_ = 0;
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
    
    
    
    if( measurement_laser_.size() == 0) return;
    
    figure_search_space_.clear();
    
    for ( int r = 0; r < Vs_.rows; r++ ) {
        for ( int c = 0; c < Vs_.cols; c++ ) {
            double j = r * 0.1;
            double i = -90.0 + c * 2.0;
            
            if(Vs_.at<uint8_t>(r,c) == 1){
                Point2D d(i, j);
                figure_search_space_.circle ( d, 5, Figure::niceRed, 2 );
            }
        }
    }
    
    double dt = duration_last_update_.total_microseconds() /1000000.;
    double max_acc_tra = 1.5 * dt;
    double max_acc_rot = 30.0 * dt;
    double v = cmd_.v();
    double w = cmd_.w() * (180.0 / M_PI);
    
    Point2D dw_corners[] = {
        Point2D(w - max_acc_rot, v - max_acc_tra),
        Point2D(w - max_acc_rot, v + max_acc_tra),
        Point2D(w + max_acc_rot, v + max_acc_tra),
        Point2D(w + max_acc_rot, v - max_acc_tra),
    };
    
    figure_search_space_.circle ( Point2D(w, v), 5, 0, 2 );
    figure_search_space_.line(dw_corners[0], dw_corners[1], 0);
    figure_search_space_.line(dw_corners[1], dw_corners[2], 0);
    figure_search_space_.line(dw_corners[2], dw_corners[3], 0);
    figure_search_space_.line(dw_corners[3], dw_corners[0], 0);
    
    cv::imshow ( figure_search_space_.title(),figure_search_space_.view() );
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
            // only drive forward if looking towards the goal (and reduce velocity approaching the goal)
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

Point2D dist(double v, double w, double dt) {
    double x, y;
    
    if(abs(w) < 0.01){
        x = v*dt;
        y = 0.0;
    } else {
        x = (v/w) * sin(w*dt);
        y = (v/w) - (v/w) * cos(w*dt);
    }
    
    return Point2D(x, y);
}

void Goto::fill_search_space(){
    if( measurement_laser_.size() == 0) return;
    
    double dt = duration_last_update_.total_microseconds() /1000000.;
    
    double laser_ang_min = measurement_laser_[0].angle;
    double laser_ang_max = measurement_laser_[measurement_laser_.size() - 1].angle;
    double laser_and_range = laser_ang_max - laser_ang_min;
    
    for ( int r = 0; r < Vs_.rows; r++ ) {
        for ( int c = 0; c < Vs_.cols; c++ ) {
            double j = r * 0.1;
            double i = -90.0 + c * 2.0;
            
            Point2D p = dist(j, i * (M_PI / 180.0), dt);
            
            Point2D p_laser = p - measurement_laser_.pose2d().position();
            
            // get index of closest lazer beam to that direction
            size_t index = (p_laser.angle() - laser_ang_min) / laser_and_range * (measurement_laser_.size() - 1);
            
            //TODO: update this condition to proper one from slide 7
            if(measurement_laser_[index].length < p_laser.radius()){
                //not allowed
                Vs_.at<uint8_t>(r,c) = 1;
            }else{
                //allowed
                Vs_.at<uint8_t>(r,c) = 0;
            }
        }
    }
}

double Goto::NF(double v, double w, Pose2D target) {
    double a = 1.0, b = 100.0, c = 1.0, d = 100.0;
    double ret;
    
    Point2D p = dist(v, w, 1.0);
    
    double distance = (pred_pose_.tf() * p).distanceTo(target.position());
    
    ret = a * v + b / distance /* + c * ... + d * ... */;
    
    return ret;
}

Command Goto::select_from_dw(Pose2D target)
{    
    double dt = duration_last_update_.total_microseconds() /1000000.;
    double max_acc_tra = 1.5 * dt;
    double max_acc_rot = M_PI/6.0 * dt;
    double v = cmd_.v();
    double w = cmd_.w();
    
    int r_start = std::max(0, (int)((v - max_acc_tra) / 0.1));
    int r_end   = std::min(Vs_.rows, (int)((v + max_acc_tra) / 0.1));
    int c_start = std::max(0, (int)((w + M_PI/2.0 - max_acc_rot) / 0.0349066));
    int c_end   = std::min(Vs_.cols, (int)((w + M_PI/2.0 + max_acc_rot) / 0.0349066));
    
    //ROS_INFO("row - s: %d, e: %d, n: %d", r_start, r_end, Vs_.rows);
    //ROS_INFO("col - s: %d, e: %d, n: %d", c_start, c_end, Vs_.cols);
    
    double max = -1.0;
    double v_max = 0.0, w_max = 0.0;
    
    for ( int r = r_start; r < r_end; r++ ) {
        for ( int c = c_start; c < c_end; c++ ) {
            double j = r * 0.1;
            double i = -M_PI/2.0 + c * 0.0349066;
            
            if(Vs_.at<uint8_t>(r,c) == 0){
                double temp = NF(j, i, target);
                
                if(temp > max){
                    max = temp;
                    v_max = j;
                    w_max = i;
                }
            }
        }
    }
    
    return Command(v_max, w_max);
}

void Goto::bug2() {
    /**
    * @ToDo 4.3 Avoid obstacle
    **/
    if(loop_count_%2 != 0) return;

    if(goal_set_ && (!config_.use_path || path_set_) && updateTimestamp(ros::Time::now().toBoost())) {
        Pose2D target;

        if(config_.use_path) {
            target = path_next_step_;
        } else {
            target = goal_;
        }
        
        if(abs(moro::angle_difference(target.theta(), pred_pose_.theta())) < 0.1 &&
            target.position().distanceTo(pred_pose_.position()) < 0.5
        ) {
            ROS_INFO("Target reached");
            cmd_.set(0.0, 0.0);
            goal_set_ = false;
        } else {
        
            fill_search_space();
            
            cmd_ = select_from_dw(target);
        }
    }
}

///@return true on successful update, false on first use and if t is in the past
bool Goto::updateTimestamp ( const boost::posix_time::ptime& t )  {
    if ( timestamp_last_update_.is_not_a_date_time() ) timestamp_last_update_ = t;
    if ( timestamp_last_update_ < t ) {
        duration_last_update_ = t - timestamp_last_update_;
        timestamp_last_update_ = t;
        return true;
    } else {
        return false;
    }
}
