#ifndef NAIVE_ROBOT_STATE_COLLECTION
#define NAIVE_ROBOT_STATE_COLLECTION
#include "planning/robot_state_collection.hpp"
#include <map>

namespace planning {

    class NaiveRobotStateCollection: public RobotStateCollection {

        public:
            NaiveRobotStateCollection();
            std::vector<RobotState*> get_states();
            RobotState* nearest(RobotState*);
            std::vector<RobotState*> states_within_radius(RobotState*, double radius);
            void insert(RobotState*);
            int size();
        private:
            std::vector<RobotState*> robot_states;
    };
}
#endif