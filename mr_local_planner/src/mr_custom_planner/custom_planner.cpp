#include "mr_custom_planner/custom_planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace moro;

CustomPlanner::CustomPlanner ( const std::string &ns )
    : action_state_ ( ActionState::NA ) {
}

/// Implements AI as in the local_planner variant, features only the GoTo "bug1" mode
void CustomPlanner::ai() {
    cmd_.set ( 1.0, 0 );
    std::cout << "AI" << std::endl;
    std::cout << cmd_ << std::endl;
}