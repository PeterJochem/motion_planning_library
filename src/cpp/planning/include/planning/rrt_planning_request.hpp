#ifndef RRT_PLANNING_PARAMETERS
#define RRT_PLANNING_PARAMETERS
#include "planning/planning_request.hpp"

namespace planning {

    class RRTPlanningRequest : public PlanningRequest {

        public:
            RRTPlanningRequest(RobotState* start_state, RobotState* goal_state, int max_num_states_in_graph, float delta);
            int max_num_states_in_graph;
            float delta;
        protected:

        private:

    };
}
#endif