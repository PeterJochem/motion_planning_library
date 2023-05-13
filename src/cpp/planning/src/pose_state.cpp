#include "planning/pose_state.hpp"

namespace planning {

    PoseState::PoseState() {

    }

    PoseState::PoseState(std::vector<float> configuration): configuration(configuration) {

    }

    int PoseState::dimension() {
        return configuration.size();
    }
 
    PoseState random(Robot::Robot1*) {
        return PoseState();
    }

    float PoseState::distance(RobotState* other) {
        return 0.0;
    }

}