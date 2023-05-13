#include "planning/rrt_planning_request.hpp"


namespace planning {

    RRTPlanningRequest::RRTPlanningRequest(RobotState* start_state, RobotState* goal_state, int max_num_states_in_graph, float delta): PlanningRequest(start_state, goal_state) {
        this->max_num_states_in_graph = max_num_states_in_graph;
        this->delta = delta;
    }
}