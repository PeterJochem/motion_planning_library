#include "planning/joint_state.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"

namespace planning {

    JointState::JointState() {

    }

    JointState::JointState(std::vector<float> configuration): configuration(configuration)  {

    }

    std::vector<float> JointState::get_configuration() {
        return configuration;
    }

    int JointState::dimension() {
        return configuration.size();
    }

    bool JointState::is_legal(Robot::Robot1* robot, FCLRobotInternalCollisionChecker checker) {
        
        // FIX ME - check that the joint angles are satisfied

        // this checks the joint state that the robot is currently in
        robot->set_joint_angles(configuration);
        bool has_internal_collision = checker.check();


        return !has_internal_collision;
    }

    JointState random(Robot::Robot1* robot) {
        robot->random_joint_angles();
    }
}