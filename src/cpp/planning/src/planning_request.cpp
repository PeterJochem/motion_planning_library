#include "planning/planning_request.hpp"

namespace planning {

    PlanningRequest::PlanningRequest(RobotState* start_state, RobotState* goal_state) {
        this->start_state = start_state;
        this->goal_state = goal_state;
    }

    RobotState* PlanningRequest::getStartState() {
        return this->start_state;
    }

    RobotState* PlanningRequest::getGoalState() {
        return this->goal_state;
    }

}