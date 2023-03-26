#ifndef STATE_SPACE_SAMPLER
#define STATE_SPACE_SAMPLER
#include "planning/robot_state.hpp"

namespace planning {

class StateSpaceSampler {

    public:
        StateSpaceSampler(Robot::Robot1*, RobotState*, RobotState*);
        RobotState* sample();

    protected:

    private:

};
}
#endif