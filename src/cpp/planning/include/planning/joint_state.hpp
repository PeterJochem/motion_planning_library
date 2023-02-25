#include "planning/robot_state.hpp"

namespace planning {

    class JointState: public RobotState {

        public:
            JointState();
            JointState(std::vector<float>);
            std::vector<float> configuration();

        private:

    };
}