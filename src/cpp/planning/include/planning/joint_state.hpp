#ifndef JOINT_STATE
#define JOINT_STATE
#include "planning/robot_state.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"

namespace planning {

    class JointState: public RobotState {

        public:
            JointState();
            JointState(std::vector<float>);
            std::vector<float> get_configuration();
            int dimension();
            bool is_legal(Robot::Robot1*, RobotInternalCollisionChecker*);
            float distance(JointState&);
            friend JointState operator+(JointState &lhs, JointState &rhs);
            friend JointState operator-(JointState &lhs, JointState &rhs);
            
        private:
            std::vector<float> configuration;
    };

    JointState random(Robot::Robot1*);
}
#endif