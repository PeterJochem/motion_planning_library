#include "planning/robot_state.hpp"
#include <tuple>

namespace planning {

    class PoseState: public RobotState {

        public:
            PoseState();
            PoseState(std::vector<float>);
            std::vector<float> get_configuration();
            int dimension();

        private:
            std::vector<float> configuration;
    };

    PoseState random(Robot::Robot1*);
}