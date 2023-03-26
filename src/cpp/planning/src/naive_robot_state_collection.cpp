#include "planning/naive_robot_state_collection.hpp"

namespace planning {

    NaiveRobotStateCollection::NaiveRobotStateCollection() {

    }

    std::vector<RobotState*> NaiveRobotStateCollection::get_states() {
        return robot_states;
    }

    RobotState* NaiveRobotStateCollection::nearest(RobotState* target_state) {

        if (robot_states.size() == 0) {
            throw std::runtime_error("The naive robot state collection is empty. Cannot compute a nearest robot state.");
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