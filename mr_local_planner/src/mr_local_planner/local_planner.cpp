#include "mr_local_planner/local_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

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

    cv::putText(figure_local_.background(), "Alexander Lampalzer",
                cv::Point(figure_local_.view().cols - 250, figure_local_.view().rows - 10),
                cv::FONT_HERSHEY_PLAIN, 1, Figure::black, 1, cv::LINE_AA);
}


void LocalPlanner::plot() {
    if (config_.plot_data) plotLocal();
    cv::waitKey(10);
}

void LocalPlanner::plotLocal() {
    figure_local_.clear();

#if PLANNER_EXERCISE >= 30
#else
    for (int i = 0; i < measurement_laser_.size(); ++i) {
        if (-0.78 <= measurement_laser_[i].angle && measurement_laser_[i].angle <= 0.78) {
            figure_local_.circle(measurement_laser_[i].end_point, 1, Figure::green);
        } else {
            figure_local_.circle(measurement_laser_[i].end_point, 1, Figure::red);
        }
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
    double u = 0.8;
    double v = 0;

    if (!measurement_laser_.empty()) {
        double min = measurement_laser_.range_max();

        for (int i = 0; i < measurement_laser_.size(); ++i) {
            if (-0.78 <= measurement_laser_[i].angle && measurement_laser_[i].angle <= 0.78) {
                if (measurement_laser_[i].length < min) {
                    min = measurement_laser_[i].length;
                }
            }
        }

        if (min < 1.0) {
            u = 0;
            v = 0.5;
        }
    }

    cmd_.set(u, v);
}

void LocalPlanner::wanderer2() {
    double u = 0.3;
    double v = 0;

    if (!measurement_laser_.empty()) {
        double min = measurement_laser_.range_max();

        for (int i = 0; i < measurement_laser_.size(); ++i) {
            if (-0.78 <= measurement_laser_[i].angle && measurement_laser_[i].angle <= 0.78) {
                if (measurement_laser_[i].length < min) {
                    min = measurement_laser_[i].length;
                }
            }
        }

        if (min < 1.0) {
            u = 0;
            v = 0.3;
        }
    }

    cmd_.set(u, v);
}


void LocalPlanner::bug1() {
    cmd_.set(0.25, 0);
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
