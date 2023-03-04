#include "planning/rrt.hpp"
#include "robots/UR5.hpp"
#include "planning/joint_state.hpp"

int main() {


    Robot::Robot1* robot = new Robot::UR_5();
    auto start_state = planning::JointState({0., 0., 0., 0., 0., 0.});
    auto goal_state = planning::JointState({1., 1., 1., 1., 1., 1.});
    planning::PlanningRequest planning_request = planning::PlanningRequest(&start_state, &goal_state);


    auto rrt = planning::RRT(robot, planning_request);
    auto robot_states = rrt.build_tree();

    std::cout << robot_states->size() << std::endl;

    return 0;
}