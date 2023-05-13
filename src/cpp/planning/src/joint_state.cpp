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

    bool JointState::is_legal(Robot::Robot1* robot, RobotInternalCollisionChecker* checker) {
        
        // FIX ME - check that the joint angles are satisfied
        robot->set_joint_angles(configuration);
        bool has_internal_collision = checker->check();
        return !has_internal_collision;
    }

    JointState random(Robot::Robot1* robot) {
        robot->random_joint_angles();
    }

    JointState operator+(JointState &lhs, JointState &rhs) {
        
        if (lhs.dimension() != rhs.dimension()) {
            throw std::runtime_error("Cannot add joint states which are of different dimensions.");
        }

        auto lhs_configuration = lhs.get_configuration();
        auto rhs_configuration = rhs.get_configuration();
        std::vector<float> sum = std::vector<float>();
        sum.reserve(lhs.dimension());
        for (int i = 0; i < lhs.dimension(); i++) {
            sum.push_back(lhs_configuration[i] + rhs_configuration[i]);
        }

        return JointState(sum);
    }

    JointState operator-(JointState &lhs, JointState &rhs) {

        if (lhs.dimension() != rhs.dimension()) {
            throw std::runtime_error("Cannot add joint states which are of different dimensions.");
        }

        
        auto lhs_configuration = lhs.get_configuration();
        auto rhs_configuration = rhs.get_configuration();
        std::vector<float> sum = std::vector<float>();
        sum.reserve(lhs.dimension());
        for (int i = 0; i < lhs.dimension(); i++) {
            sum.push_back(lhs_configuration[i] - rhs_configuration[i]);
        }

        return JointState(sum);
    }



    float JointState::distance(JointState& other) {
        
        if (dimension() != other.dimension()) {
            // ...
        }

        float distance = 0.0;
        auto configuration = get_configuration();
        auto other_configuration = other.get_configuration();
        for (int i = 0; i < dimension(); i++) {
            distance += pow(configuration[i] - other_configuration[i], 2);
        }

        return sqrt(distance);
    }
    

}