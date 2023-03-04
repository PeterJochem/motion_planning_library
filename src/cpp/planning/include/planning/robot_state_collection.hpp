#ifndef ROBOT_STATE_COLLECTION
#define ROBOT_STATE_COLLECTION
#include "planning/robot_state.hpp"
#include "planning/utilities.hpp"

namespace planning {

    class RobotStateCollection {

        public:
            virtual void add_edge(RobotState*, RobotState*) = 0;
            virtual std::vector<RobotState*> get_states() = 0;
            virtual RobotState* nearest(RobotState*) = 0;
            virtual void insert(RobotState*) = 0;
            virtual int size() = 0;
        
        protected:
            std::vector<RobotState*> robot_states;
        
        private:

    };
}
#endif