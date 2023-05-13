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

    std::vector<RobotState*> NaiveRobotStateCollection::states_within_radius(RobotState* state, double radius)  {

        if (robot_states.size() == 0) {
            return {};
        }
        
        std::vector<RobotState*> results;
        std::copy_if(robot_states.begin(), robot_states.end(), std::back_inserter(results), [state, radius](auto other_state) {return sum_square_errors(state, other_state) < radius;});
        return results;
    }


    void NaiveRobotStateCollection::insert(RobotState* state) {
        robot_states.push_back(state);
    }
    
    
    int NaiveRobotStateCollection::size() {
        return robot_states.size();
    }
}