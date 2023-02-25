#include "planning/robot_state.hpp"
#include <tuple>

namespace planning {

    class PoseState: public RobotState {

        public:
            PoseState();
            PoseState(std::vector<float>);

        private:

    };

    PoseState random(Robot::Robot1*);
}