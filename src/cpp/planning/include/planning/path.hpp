#include "planning/robot_state.hpp"
#include <vector>

namespace planning {
    /* A series of kinematic states for a robot to pass through. No notion of when the robot is supposed to be at each state.
    */
    class Path {

        public:
            Path();
        protected:
            std::vector<RobotState*> robot_states;
        private:


    };
}