#include "planning/joint_state.hpp"
#include "planning/naive_robot_state_collection.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"
#include "planning/rrt_connect.hpp"


namespace planning {


    RRTConnect::RRTConnect(Robot::Robot1* robot, RRTPlanningRequest* request): RRTBase(robot), request(request) {


    }

    Path* RRTConnect::solve() {

        auto tree_rooted_at_start = robot_states;
        RobotStateCollection* tree_rooted_at_goal = new NaiveRobotStateCollection();

        tree_rooted_at_start->insert(request->getStartState());
        tree_rooted_at_goal->insert(request->getGoalState());

        auto current_tree = tree_rooted_at_start;
        auto secondary_tree = tree_rooted_at_goal;

        planning::JointState* primary_tree_linking_state = nullptr;
        planning::JointState* secondary_tree_linking_state = nullptr;

        for (int i = 0; i < request->max_num_states_in_graph; i++) {
            
            // Generate a random state.
            planning::RobotState* random_joint_state = new planning::JointState(robot->random_joint_angles());

            if (i % 20 == 0) {
                random_joint_state = new planning::JointState((request->getGoalState())->get_configuration());
            }

            // Find the nearest neighbor.
            planning::RobotState* nearest;
            nearest = current_tree->nearest(random_joint_state);
           
            // Compute the new state for the primary tree.
            auto delta_vector = subtract(((planning::JointState*)random_joint_state)->get_configuration(), ((planning::JointState*)nearest)->get_configuration());    
            scale(delta_vector, request->delta);
            planning::JointState* new_state_for_primary_tree = new JointState( add(nearest->get_configuration(), delta_vector) );

            // Insert the new state if it is legal.
            if (new_state_for_primary_tree->is_legal(robot, internal_collision_checker)) {

                current_tree->insert(new_state_for_primary_tree);
                parent_lookup[new_state_for_primary_tree] = nearest;
            
                // Check if we are done.
                auto secondary_tree_nearest = secondary_tree->nearest(new_state_for_primary_tree);
                auto distance = ((planning::JointState*)secondary_tree_nearest)->distance(new_state_for_primary_tree);

                if (distance < 0.25) {
                    std::cout << "The RRT algorithm terminated." << std::endl;
                    //parent_lookup[secondary_tree_nearest] = new_state_for_primary_tree;
                    
                    primary_tree_linking_state = new_state_for_primary_tree;
                    secondary_tree_linking_state = (planning::JointState*)secondary_tree_nearest;
                    break;
                }
            }

            /*
            // Compute the new state for the secondary tree.
            auto nearest_in_secondary_tree = secondary_tree->nearest(new_state_for_primary_tree);

            auto delta_vector_ = subtract(((planning::JointState*)new_state_for_primary_tree)->get_configuration(), ((planning::JointState*)nearest_in_secondary_tree)->get_configuration());    
            scale(delta_vector, request->delta);
            planning::JointState* new_state_for_secondary_tree = new JointState( add(nearest_in_secondary_tree->get_configuration(), delta_vector_) );

            // Insert the new state if it is legal.
            if (new_state_for_secondary_tree->is_legal(robot, internal_collision_checker)) {

                //secondary_tree->insert(new_state_for_secondary_tree);
                //parent_lookup[new_state_for_secondary_tree] = nearest_in_secondary_tree;


                // Check if we are done.
                auto primary_tree_nearest = current_tree->nearest(new_state_for_secondary_tree);
                auto distance = ((planning::JointState*)primary_tree_nearest)->distance(new_state_for_secondary_tree);

                if (distance < 0.05) {
                    std::cout << "The RRT algorithm terminated." << std::endl;
                    //parent_lookup[secondary_tree_nearest] = new_state_for_primary_tree;

                    //primary_tree_linking_state = (planning::JointState*)primary_tree_nearest;
                    //secondary_tree_linking_state = (planning::JointState*)nearest_in_secondary_tree;
                    //break;
                }
            }
            */

            // Swap the trees.
            auto prior_current_tree = current_tree;
            current_tree = secondary_tree;
            secondary_tree = prior_current_tree;


            // Junk but useful for manual testing.
            auto goal = *((JointState*)(request->getGoalState()));
            auto tmp2 = robot_states->nearest(request->getGoalState());
            auto distance = ((planning::JointState*)tmp2)->distance(&goal);
            std::cout << i << ": The nearest distance is " << distance << std::endl;
        }


        // Convert the robot state collection to a path.
        // 1. Find the path from the start state to the primary_tree_linking_state
        std::vector<JointState> path_from_start_states = std::vector<JointState>();
        RobotState* current = (RobotState*)(primary_tree_linking_state);
        while (current != nullptr) {
            path_from_start_states.push_back(*((JointState*)(current)));
            current = (parent_lookup[((RobotState*)current)]);
        }

        // 2. Find the path from the secondary tree linking state to the goal.
        std::vector<JointState> path_from_goal_states;
        current = (RobotState*)(request->getGoalState());
        while (current != nullptr) {
            path_from_goal_states.push_back(*((JointState*)(current)));
            current = (parent_lookup[((RobotState*)current)]);
        }

        path_from_goal_states.push_back(*((JointState*)(secondary_tree_linking_state)));
        std::reverse(path_from_goal_states.begin(), path_from_goal_states.end());

        // 3. Join the two paths together.
        path_from_start_states.insert(path_from_start_states.end(), path_from_goal_states.begin(), path_from_goal_states.end());

        // Get the list of joint names
        auto links = robot->get_links();
        std::vector<std::string> names = std::vector<std::string>();
        for (int i = 0; i < links.size(); i++) {
            names.push_back(links[i].get_transform()->get_parent().get_name());
        }

        
        return new Path(path_from_start_states, names);
    }
}
