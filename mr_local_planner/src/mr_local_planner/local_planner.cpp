#include "mr_local_planner/local_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace cv;
using namespace moro;

std::map<LocalPlanner::ControlMode, std::string> LocalPlanner::ControlModeName_ = {
        {DEMO,       "DEMO"},
        {STOP,       "STOP"},
        {WANDERER1,  "WANDERER1"},
        {WANDERER2,  "WANDERER2"},
        {BUG1,       "BUG1"},
        {BUG2,       "BUG2"},
        {TANGENSBUG, "TANGENSBUG"}
};

LocalPlanner::LocalPlanner(const std::string &ns)
        : loop_count_(0), figure_local_(ns + ", Local View"), action_state_(ActionState::NA) {

}

void LocalPlanner::init() {

    figure_local_.init(config_.map_pix_x, config_.map_pix_y,
                       config_.map_min_x, config_.map_max_x,
                       config_.map_min_y, config_.map_max_y,
                       config_.map_rotation + M_PI / 2.0,
                       config_.map_grid_x, config_.map_grid_y);

    cv::putText(figure_local_.background(), ControlModeName_[(ControlMode) config_.mode],
                cv::Point(5, figure_local_.view().rows - 10),
                cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA);

    /**
     * @ToDo Wanderer
     * change the Maxima Musterfrau to your name
     **/
    cv::putText(figure_local_.background(), "Team blue", // initial version by "Luigi Berducci"
                cv::Point(figure_local_.view().cols - 250, figure_local_.view().rows - 10),
                cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA);
}


void LocalPlanner::plot() {
    if (config_.plot_data) plotLocal();
    cv::waitKey(10);
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
    for (int i = 0; i < measurement_laser_.size(); i++) {
        MeasurementLaser::Beam beam = measurement_laser_[i];
        figure_local_.circle(beam.end_point, 1, Figure::red, 2);
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
    cv::imshow(figure_local_.title(), figure_local_.view());
}

/**
 * @note these are the different robot behaviours.
 * At least one implementation is required for Exercise 2 Part 2
 */
void LocalPlanner::ai() {
    switch (config_.mode) {
        case STOP:
            cmd_.set(0, 0);
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
        case GOTO:
            goto_plan();
            break;
        default:
            cmd_.set(0, 0);
    }
    loop_count_++;
}


/**
* Demo
**/
void LocalPlanner::demo() {
    double v = 0.0, w = 0.0;
    if (measurement_laser_.empty()) {
        v = 0.0, w = -0.1;
    } else {
        if (measurement_laser_[measurement_laser_.size() / 4].length < 1.0) {
            w = 0.4;
        } else {
            v = 0.4;
        }
    }
    cmd_.set(v, w);
}

void LocalPlanner::wanderer1() {
    /**
    * @ToDo Wanderer
    * Implementation of a simple controller "à la Roomba"
    *     1. check if there is enough free space (ie., a corridor) in front of the robot
    *     2. if corrifor is free: go straight
    *     3. otherwise: start a rotation (random left/right) until new free corridor
    **/

    // parameters    
    const int raysHalfFOV = 90;                                       // half field of view (rays)
    const int beginFOV = measurement_laser_.size() / 2 - raysHalfFOV;  // first ray defining the frontal corridor
    const int endFOV = measurement_laser_.size() / 2 + raysHalfFOV;    // last ray defining the frontal corridor
    const double safetyDist = 1.5;                 // critical distance to detect an obstacle in the FOV (in meter)
    const double corridorHalfWidth = 0.4;           // used to trigger the in-place rotation

    // select action only if the previous has been completed
    double closestDistance = 100;   // distance (meter) to the closest obstacle
    if (!measurement_laser_.empty()) {
        // check if frontal corridor is free
        bool corridorIsFree = true;
        for (int i = beginFOV; i < endFOV; i++) {
            closestDistance = min(closestDistance, measurement_laser_[i].length);
            if (measurement_laser_[i].length < safetyDist) {
                double alpha = measurement_laser_[measurement_laser_.size() / 2].angle - measurement_laser_[i].angle;
                double distance = abs(measurement_laser_[i].length * sin(alpha));
                if (distance < corridorHalfWidth) {
                    corridorIsFree = false;
                }
            }
        }
        // choose action to perform
        if (corridorIsFree) {
            action_state_ = ActionState::STRAIGHT;
        } else {
            // choose new rotation only if not already performing a rotation
            if (action_state_ == ActionState::STRAIGHT) {
                if (rand() < RAND_MAX / 2)
                    action_state_ = ActionState::TURN_LEFT;
                else
                    action_state_ = ActionState::TURN_RIGHT;
            }
        }
    } else {
        if (rand() < RAND_MAX / 2)
            action_state_ = ActionState::TURN_LEFT;
        else
            action_state_ = ActionState::TURN_RIGHT;
    }

    // perform action
    std::normal_distribution<> v_noise{0.0, config_.v_std};
    std::normal_distribution<> w_noise{0.0, config_.w_std};

    double v = 0.0, w = 0.0;
    switch (action_state_) {
        case ActionState::STRAIGHT:
            if (closestDistance < safetyDist) {
                v = 0.5 * config_.v_mean + v_noise(gen);    // if close obstacle, max velocity 0.3 m/s
            } else {
                v = config_.v_mean + v_noise(gen);    // otherwise, max velocity 0.8 m/s
            }
            w = 0.0;
            break;
        case ActionState::TURN_LEFT:
            v = 0.0;
            w = config_.w_mean + w_noise(gen);
            break;
        case ActionState::TURN_RIGHT:
            v = 0.0;
            w = -(config_.w_mean + w_noise(gen));
            break;
        default:
            // do nothing
            break;
    }

    // send control command
    cmd_.set(v, w);
}

void LocalPlanner::wanderer2() {
    /**
    * @ToDo Wanderer
    * OPTIONAL: if you like you can write another one :-)
    * NOTE: not working well, especially in multi-agent setting
    **/
    double v = 0.0, w = 0.0;
    // find largest gap
    double threshold = 2.0;
    int minSizeGap = 10;
    struct Gap {
        int begin;
        int end;
        int size;
        int center;

        Gap(int b, int e) {
            begin = b;
            end = e;
            center = begin + (end - begin) / 2;
            size = end - begin;
        }
    };

    std::vector<Gap> gaps;

    int beginGap = -1, endGap = -1;
    int n = measurement_laser_.size();
    for (int i = 45; i < n - 45; i++) {
        if (measurement_laser_[i].length >= threshold) {
            if (beginGap < 0) {
                beginGap = i;
            }
            if (measurement_laser_[i + 1].length < threshold) {
                endGap = i;
                if (endGap - beginGap >= minSizeGap) {
                    Gap gap(beginGap, endGap);
                    gaps.push_back(gap);
                }
                // reset gap
                beginGap = -1;
                endGap = -1;
            }
        }
    }
    // if no gap, start rotating 
    if (gaps.empty()) {
        v = 0.0;
        w = 0.5;
    } else {
        // find closer gap to the center
        int distToCenter = 1000;
        Gap bestGap(0, 0);
        for (Gap gap: gaps) {
            if (abs(n / 2 - gap.center) < distToCenter) {
                distToCenter = abs(n / 2 - gap.center);
                bestGap = gap;
            }
            if ((n / 2 >= gap.begin) && (n / 2 <= gap.end)) {
                bestGap = gap;
                break;
            }
        }
        double angle = measurement_laser_[bestGap.center].angle;
        w = min(0.5, max(-0.5, angle)); // clip angle within limits
        v = 0.2 + 0.6 * (1 - abs(w) / 0.5);
    }
    cmd_.set(v, w);
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

void LocalPlanner::goto_plan() {
    /// wait to have a path
    if (path_.empty()) return;

    double lookahead = 1.0;

    ROS_INFO_STREAM("Mode: " << action_state_);
    switch (action_state_) {
        case WAIT:
            break;
        case TRACK:
            if (this->nextWaypointReachable(lookahead)) {
                path_tracking(lookahead);
            } else {
                ROS_INFO_THROTTLE(1, "can't see path, using local planner");
                this->alternative_planner();
            }
            break;
        case ALIGN:
            final_alignment();
            break;
        case REPLAN:
            break;
    }
}

int LocalPlanner::getNextWaypointID(const double lookahead) {
    // find closest waypoint
    int closestID = -1;
    double minDist = 100;
    Pose2D& pose = this->transform_;
    Point2D position = pose.position();
    for (size_t i = 0; i < path_.size(); i++) {
        Pose2D waypoint = path_[i];
        double dist = position.distanceTo(waypoint.position());
        if (dist < minDist) {
            minDist = dist;
            closestID = i;
        }
    }
    // find waypoint which is > lookahead distance from closest one
    while (minDist < lookahead && closestID < path_.size() - 1) {
        Point2D current = path_[closestID].position();
        Point2D next = path_[closestID + 1].position();
        minDist += current.distanceTo(next);
        closestID++;
    }
    return closestID;
}


void LocalPlanner::path_tracking(double lookahead) {
    /**
     * Geometric algorithm to track path: control steer and speed w.r.t. the closest waypoint > lookahead distance.
     * Simple proportional control, no fancy steering (e.g., pure pursuit).
     *
     * use path_, odom_
     *
     */
    // parameters
    double maxsteer = 0.5, maxspeed = 0.8;
    const double zeroSpeedTolerance = 0.1;  // to change state to final alignment
    Pose2D& pose = this->transform_;

    // transform selected target to robot frame
    int closestID = getNextWaypointID(lookahead);
    targetWaypoint_ = path_[closestID];     // used in visualization
    auto target_robot = pose.tf().inv() * targetWaypoint_.position();

    // compute lateral control
    double steer = atan2(target_robot.y(), target_robot.x());

    // compute longitudinal control
    double speed = 0;
    double dist = sqrt(pow(target_robot.x(), 2) + pow(target_robot.y(), 2));
    double normDist = min(dist, lookahead) / lookahead;
    if (dist > lookahead) {
        // speed proportional to target and inv. prop to steering
        double normSteer = min(fabs(steer), maxsteer) / maxsteer;
        speed = maxspeed * (normDist + (1 - normSteer)) / 2;
    } else {
        // lower speed proportional to target
        speed = maxspeed / 2 * normDist;
    }
    // set command
    cmd_.set(speed, steer);
    // finally, if reached the target, align with its orientation
    if (closestID == path_.size() - 1 && speed <= zeroSpeedTolerance) {
        action_state_ = ActionState::ALIGN;
    }
}

void LocalPlanner::final_alignment() {
    /// perform inplace rotation to align the robot heading to the final pose
    /// when reach almost zero error, move to "WAIT" state
    const double ks = 0.5;  // proportional gain
    const double errTolerance = 0.01;
    double speed = 0, steer = 0;
    // compute error w.r.t. last waypoint
    Pose2D lastWaypoint = path_[path_.size() - 1];
    double normError = angle_normalize(angle_difference(this->transform_.theta(), lastWaypoint.theta())) / M_PI;
    steer = ks * normError;
    // set command
    cmd_.set(speed, -steer);
    // eventually change state
    if (abs(steer) < errTolerance ){
        action_state_ = ActionState::WAIT;
    }
    // debug
    ROS_DEBUG_STREAM("[Mode ALIGN] norm error: " << normError << ", speed : " << speed << ", steer: " << steer);
}

void LocalPlanner::updateTransform(geometry_msgs::TransformStamped& new_transform, geometry_msgs::TransformStamped& new_laser_transform) {
    transform_.set_x(new_transform.transform.translation.x);
    transform_.set_y(new_transform.transform.translation.y);

    tf2::Quaternion quat;
    tf2::convert(new_transform.transform.rotation, quat);

    double _roll, _pitch, yaw;
    tf2::Matrix3x3(quat).getEulerYPR(yaw, _pitch, _roll);
    transform_.set_theta(yaw);

    laser_transform_.set_x(new_laser_transform.transform.translation.x);
    laser_transform_.set_y(new_laser_transform.transform.translation.y);

    tf2::Quaternion laser_quat;
    tf2::convert(new_laser_transform.transform.rotation, laser_quat);

    double _laser_roll, _laser_pitch, laser_yaw;
    tf2::Matrix3x3(laser_quat).getEulerYPR(laser_yaw, _laser_pitch, _laser_roll);
    laser_transform_.set_theta(laser_yaw);
}

bool LocalPlanner::nextWaypointReachable(double lookahead) {
    if (path_.empty())
        return false;

    const double angle_min = -2.3561945;
    const double angle_max = 2.3561945;
    const double angle_increment = 0.0175181739;

    int waypoint_index = this->getNextWaypointID(lookahead);
    Pose2D target_waypoint = this->path_[waypoint_index];
    Point2D target_waypoint_laser = this->laser_transform_.tf().inv() * target_waypoint.position();
    Polar2D target_waypoint_polar(target_waypoint_laser);

    double distance = target_waypoint.position().distanceTo(this->transform_.position());
    double ray_calc = (target_waypoint_polar.alpha() - angle_min) / angle_increment;

    if (ray_calc < 0.0 || ray_calc > (double) this->measurement_laser_.size()) {    // we can't see the point so we don't even have to try
        return false;
    }

    int ray = (int) std::round(ray_calc);
    for (size_t i = std::max(0, ray - 10); i < std::min((int) this->measurement_laser_.size(), ray + 10); ++i) {
        if (this->measurement_laser_[i].length - distance < 0.3) {
            return false;
        }
    }
    return true;
}

void LocalPlanner::alternative_planner() {
    this->cmd_.set(0.0, 0.0);
}
