#include "planning/robot_state.hpp"


class JointState: public RobotState {

    public:
        JointState();
        JointState(std::vector<float>);

    private:

};