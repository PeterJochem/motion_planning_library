#include "planning/joint_state.hpp"

namespace planning {

    JointState::JointState() {

    }

    JointState::JointState(std::vector<float> configuration): configuration(configuration)  {

    }

    std::vector<float> JointState::get_configuration() {
        return configuration;
    }
}