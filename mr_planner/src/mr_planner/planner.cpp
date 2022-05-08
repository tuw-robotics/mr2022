#include "mr_planner/planner.h"
#include <opencv2/core/core.hpp>
#include <boost/concept_check.hpp>
#include <iostream>

using namespace cv;
using namespace moro;

std::map<Planner::ControlMode, std::string> Planner::ControlModeName_ = {
        {STRAIGHT,       "STRAIGHT"},
        {STRAIGHT_AVOID, "STRAIGHT AVOID"},
        {PATH,           "FOLLOW PATH"},
        {STOP,           "STOP"}
};

Planner::Planner(const std::string &ns)
        : loop_count_(0), action_state_(ActionState::NA) {

}

void Planner::init() {

}


void Planner::ai(Pose2D world_pos) {
    switch (config_.mode) {
        case STOP:
            cmd_.set(0, 0);
            break;
        case STRAIGHT:
            straight(world_pos);
            break;
        case STRAIGHT_AVOID:
            straightAvoid(world_pos);
            break;
        case PATH:
            path(world_pos);
            break;
    }
    loop_count_++;
}

void Planner::straight(Pose2D world_pos) {
    float v = 0;
    float w = 0;
    Point2D goal_vector;
    float dist = 0;
    float angle = 0;

    switch (action_state_) {
        case (NA):
            break;
        case (INIT):
            goal_vector = Point2D(goal_.get_x()-start_.get_x(), goal_.get_y()-start_.get_y());
            subgoal_rotation_ = goal_vector.angle();

            action_state_ = ROTATION;
            break;
        case (FORWARD):

            //P control for w (should be ~0) and v.
            w = config_.velocity_p * (subgoal_rotation_ - world_pos.get_theta());

            dist = world_pos.position().distanceTo(goal_.position());

            v = 0.1 * dist;

            //Limit v.
            if (v > 0.5) v = 0.5;

            //Small motion command <-> small error to goal.
            //If small enough, we are at final goal. Start rotating towards final position.

            if (abs(dist) < config_.distance_threshold) {
                subgoal_rotation_ = goal_.get_theta();
                action_state_ = FINAL_ROTATION;
            }
            break;

        case (ROTATION):
            //P control for w
            angle = (subgoal_rotation_ - world_pos.get_theta());
            w = config_.angular_velocity_p * angle;

            //Limit turn signal
            if (w > 0.5) w = 0.5;
            if (w < -0.5) w = -0.5;

            //Small w <-> small error to rotation goal.
            //If small enough, start moving towards position goal.
            if (abs(angle) < config_.angle_threshold) {
                action_state_ = FORWARD;
            }
            break;
        case (FINAL_ROTATION):
            //P control for w.

            angle = (subgoal_rotation_ - world_pos.get_theta());
            w = config_.angular_velocity_p * angle;

            //Limit turn signal
            if (w > 0.5) w = 0.5;
            if (w < -0.5) w = -0.5;

            if (abs(angle) < config_.angle_threshold) {
                //Close enough to goal, finish.
                action_state_ = NA;
            }
            break;

    }
    cmd_.set(v, w);
}

void Planner::straightAvoid(Pose2D world_pos) {

}

void Planner::path(Pose2D world_pos) {

}
