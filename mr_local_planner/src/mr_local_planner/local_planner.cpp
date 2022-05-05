#include "mr_local_planner/local_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <tf2/convert.h>

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
    cv::putText(figure_local_.background(), "Luigi Berducci",
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
            pure_pursuit();
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

double LocalPlanner::estimateOpenCorridor(int pivot, int nHalfRays) {
    /* 
     * Consider a corridor as centered around a `pivot` ray with a half-width `nHalfRays`,
     * we can estimate if the corridor is clean by computing the average length of its rays.
     */
    double meanDistance = 0.0;
    for (int i = pivot - nHalfRays; i < pivot + nHalfRays; i++)
        meanDistance += measurement_laser_[i].length;
    meanDistance /= 2 * nHalfRays;
    return meanDistance;
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

void LocalPlanner::pure_pursuit() {
    /**
     * Pure Pursuit algorithm.
     * use path_, odom_
     *
     * reference: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
     */
     if (path_.size() == 0) return;
     // parameters
     double lookahead = 1.0, wheelbase = 0.1;
    // find closest waypoint
    int closestID = -1;
    double minDist = 100;
    Point2D position = odom_.position();
    double theta = odom_.theta();
    for (size_t i=0; i<path_.size(); i++) {
        Point2D waypoint = path_[i];
        double dist = position.distanceTo(waypoint);
        if ( dist < minDist){
            minDist = dist;
            closestID = i;
        }
    }
    // find waypoint which is lookahead distance from closest one
    while ( minDist < lookahead && closestID < path_.size() - 1){
        Point2D current = path_[closestID];
        Point2D next = path_[closestID + 1];
        minDist += current.distanceTo(next);
        closestID++;
    }
    targetWaypoint_ = path_[closestID];
    auto target_robot = odom_.tf() * targetWaypoint_.position();

    // compute controls
    double radius = pow(lookahead, 2) / (2 * target_robot.y());
    double speed = 0.5;
    double steer = atan(wheelbase * 1 / radius);
    cmd_.set(speed, steer);
}
