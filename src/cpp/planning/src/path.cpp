#include "planning/path.hpp"

namespace planning {

    Path::Path(std::vector<JointState> robot_states, std::vector<std::string> joint_names): robot_states(robot_states), joint_names(joint_names) {
    }

    int Path::size() {
        return robot_states.size();
    }

    std::vector<JointState> Path::get_robot_states() {
        return robot_states;
    }

    std::vector<std::string> Path::get_joint_names() {
        return joint_names;
    }


}