#include "planning/joint_state.hpp"
#include "planning/rrt.hpp"
#include "planning/naive_robot_state_collection.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"
#include "transform/static_transform_tree.hpp"


namespace planning {

    RRT::RRT(Robot::Robot1* robot, PlanningRequest request): RRTBase(robot, request) {

    }

    Path* RRT::solve() {

        using namespace geometry;

        // construct transform tree
        Frame root = Frame("world");
        StaticTransformTree tree = StaticTransformTree(root);

        auto transforms = robot->get_ordered_transforms();
        for (int i = 0; i < transforms.size(); i++) {
            tree.add(transforms[i]);
        }

        FCLRobotInternalCollisionChecker checker = FCLRobotInternalCollisionChecker(*robot, &tree);
        RobotStateCollection* robot_states = new NaiveRobotStateCollection();
        // StatteSpaceSampler* state_space_sampler = new NaiveStateSpaceSampler(start state, goal state, ...);

        robot_states->insert(request.getStartState());

        int num_states = 2000;
        float delta = 0.1;
        for (int i = 0; i < num_states; i++) {
            
            // Generate a random state.
            planning::RobotState* random_joint_state = new planning::JointState(robot->random_joint_angles());
            
            if (i % 20 == 0) {
                random_joint_state = new planning::JointState((request.getGoalState())->get_configuration());
            }

            // Find the neaest neighbor.
            planning::RobotState* nearest;
            nearest = robot_states->nearest(random_joint_state);
           
            // Compute the new state.
            planning::JointState* new_state;
            
            auto delta_vector = subtract(((planning::JointState*)random_joint_state)->get_configuration(), ((planning::JointState*)nearest)->get_configuration());    
            scale(delta_vector, delta);

            new_state = new JointState( add(nearest->get_configuration(), delta_vector) );

            // Insert the new state if it is legal.
            if (new_state->is_legal(robot, checker) ) {
                robot_states->insert(new_state);
                parent_lookup[new_state] = nearest;
            }
            else {
                std::cout << "The state was in collision." << std::endl;
            }

            // Check if we are done.
            auto goal = *((JointState*)(request.getGoalState()));
            if (new_state->distance(goal) < 0.75) {
                std::cout << "The RRT algorithm terminated." << std::endl;
                parent_lookup[request.getGoalState()] = new_state;
                break;
            }
            else {
                auto tmp2 = robot_states->nearest(request.getGoalState());
                auto distance = ((planning::JointState*)tmp2)->distance(goal);
            }
        }


        // Convert the robot state collection to a path.    
        std::vector<JointState> path_states = std::vector<JointState>();
        auto tmp = (request.getGoalState());
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