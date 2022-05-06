#include "mr_planner/planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace moro;

std::map<Planner::ControlMode, std::string> Planner::ControlModeName_ = {
        {STRAIGHT,       "STRAIGHT"},
        {STRAIGHT_AVOID, "STRAIGHT AVOID"},
        {PATH, "FOLLOW PATH"},
        {STOP, "STOP"}
};

Planner::Planner(const std::string &ns)
        : loop_count_(0), action_state_(ActionState::NA) {

}

void Planner::init() {

}


void Planner::ai() {
    switch (config_.mode) {
        case STOP:
            cmd_.set(0, 0);
            break;
        case STRAIGHT:
            straight();
            break;
        case STRAIGHT_AVOID:
            straightAvoid();
            break;
        case PATH:
            path();
            break;
    }
    loop_count_++;
}

void Planner::straight(){

}

void Planner::straightAvoid(){

}

void Planner::path(){

}
