#include "planning/naive_robot_state_collection.hpp"

namespace planning {

    NaiveRobotStateCollection::NaiveRobotStateCollection() {

    }

    std::vector<RobotState*> NaiveRobotStateCollection::get_states() {
        return robot_states;
    }

    void NaiveRobotStateCollection::add_edge(RobotState* state1, RobotState* state2) {

        if (!state1 || !state2) {
            throw std::runtime_error("One of the robot states is a nullptr. Cannot add edges between states.");
        }

        // check that state1 is in the collection
        // check that state2 is in the collection

        // Add edge from state1 -> state2
        auto& state1_neighbors = edges[state1];
        state1_neighbors.push_back(state2);

        // Add edge from state2 -> state1
        auto& state2_neighbors = edges[state2];
        state1_neighbors.push_back(state1);
    }

    RobotState* NaiveRobotStateCollection::nearest(RobotState* target_state) {

        if (robot_states.size() == 0) {
            throw std::runtime_error("The naive robot state collection is empty. Cannot compute a nearet robot state.");
        }

        double min_distance = sum_square_errors(robot_states[0], target_state);
        RobotState* nearest = robot_states[0];
        for (std::vector<RobotState*>::iterator itr = robot_states.begin(); itr != robot_states.end(); itr++) {
            double distance = sum_square_errors(*itr, target_state);
            if (distance < min_distance) {
                
                nearest = *itr;
                min_distance = distance; 
            }
        }

        return nearest;
    }

    void NaiveRobotStateCollection::insert(RobotState* state) {
        robot_states.push_back(state);
    }
    
    
    int NaiveRobotStateCollection::size() {
        return robot_states.size();
    }

}