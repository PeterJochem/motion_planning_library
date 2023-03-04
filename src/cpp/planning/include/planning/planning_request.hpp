#ifndef PLANNING_REQUEST
#define PLANNING_REQUEST
#include "planning/robot_state.hpp"

namespace planning {

class PlanningRequest {

    public:
        PlanningRequest(RobotState*, RobotState*);
        RobotState* getStartState();
        RobotState* getGoalState();

    protected:

    private:
        RobotState* start_state;
        RobotState* goal_state;
};
}
#endif