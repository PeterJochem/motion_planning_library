#include "planning/joint_state.hpp"
#include "planning/rrt_star.hpp"
#include "planning/naive_robot_state_collection.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"


namespace planning {

    RRTStar::RRTStar(Robot::Robot1* robot, RRTPlanningRequest* request): RRTBase(robot), request(request) {
    }

    Path* RRTStar::solve() {

        auto initial_state = request->getStartState();
        robot_states->insert(initial_state);
        std::unordered_map<planning::RobotState*, double> costs = {{initial_state, 0.0}};

        for (int i = 0; i < request->max_num_states_in_graph; i++) {
            
            // Generate a random state.
            planning::RobotState* random_joint_state = new planning::JointState(robot->random_joint_angles());
            
            if (i % 20 == 0) {
                random_joint_state = new planning::JointState((request->getGoalState())->get_configuration());
            }

            // Find the nearest neighbor.
            planning::RobotState* nearest;
            nearest = robot_states->nearest(random_joint_state);
           
            // Compute the new state.
            planning::JointState* new_state;
            auto delta_vector = subtract(((planning::JointState*)random_joint_state)->get_configuration(), ((planning::JointState*)nearest)->get_configuration());    
            scale(delta_vector, request->delta);

            new_state = new JointState( add(nearest->get_configuration(), delta_vector) );

            // Compute the near states with some radius of the nearest node.
            double radius = 1.0;
            auto near_states = robot_states->states_within_radius(nearest, radius); 

            // Find the parent for the new state.
            auto x_min = nearest;
            auto cost_min = costs[nearest] + nearest->distance(new_state);
            for (std::vector<RobotState*>::iterator itr = near_states.begin(); itr != near_states.end(); itr++) {
                
                RobotState* neighbor  = *itr;
                auto cost = costs[neighbor] + neighbor->distance(new_state);
                if (cost < cost_min) {
                    x_min = neighbor;
                    cost_min = cost;
                }
            }

            // Insert the new state if it is legal.
            if (new_state->is_legal(robot, internal_collision_checker) ) {

                robot_states->insert(new_state);
                auto new_parent = x_min;
                parent_lookup[new_state] = new_parent;
                costs[new_state] = costs[new_parent] + new_parent->distance(new_state);

                // Check if we are done.
                auto goal = *((JointState*)(request->getGoalState()));
                if (new_state->distance(&goal) < 0.75) {
                    std::cout << "The RRT algorithm terminated." << std::endl;
                    parent_lookup[request->getGoalState()] = new_state;
                    break;
                }
            }

            // Re-wire the tree.
            for (std::vector<RobotState*>::iterator itr = near_states.begin(); itr != near_states.end(); itr++) {
                
                RobotState* near_state  = *itr;
                auto cost = costs[near_state] + near_state->distance(new_state);

                if (cost < costs[near_state]) {

                    parent_lookup[near_state] = parent_lookup[new_state];
                    costs[near_state] = cost;
                }

            }

            // Junk but useful for manual testing.
            auto goal = *((JointState*)(request->getGoalState()));
            auto tmp2 = robot_states->nearest(request->getGoalState());
            auto distance = ((planning::JointState*)tmp2)->distance(&goal);
            std::cout << i << ": The nearest distance is " << distance << std::endl;
        }


        // Convert the robot state collection to a path.    
        std::vector<JointState> path_states = std::vector<JointState>();
        auto tmp = (request->getGoalState());
        RobotState* current = (RobotState*)(tmp);
        while (current != nullptr) {
            path_states.push_back(*((JointState*)(current)));
            current = (parent_lookup[((RobotState*)current)]);
        }

        // Get the list of joint names
        auto links = robot->get_links();
        std::vector<std::string> names = std::vector<std::string>();
        for (int i = 0; i < links.size(); i++) {
            names.push_back(links[i].get_transform()->get_parent().get_name());
        }

        return new Path(path_states, names);
    }
}
