#ifndef PATH
#define PATH
#include "planning/robot_state.hpp"
#include "planning/joint_state.hpp"
#include <vector>

namespace planning {
    /* A series of kinematic states for a robot to pass through. No notion of when the robot is supposed to be at each state.
    */
    class Path {

        public:
            Path(std::vector<JointState>, std::vector<std::string>);
            int size();
            std::vector<JointState> get_robot_states();
            std::vector<std::string> get_joint_names();
        protected:
            std::vector<JointState> robot_states;
            std::vector<std::string> joint_names;
        private:


    };
}
#endif