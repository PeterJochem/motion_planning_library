#include "planning/robot_state_collection.hpp"
#include <map>

namespace planning {

    class NaiveRobotStateCollection: public RobotStateCollection {

        public:
            NaiveRobotStateCollection();
            std::vector<RobotState*> get_states();
            RobotState* nearest(RobotState*);
            void insert(RobotState*);
            int size();
        private:
            std::vector<RobotState*> robot_states;
    };
}