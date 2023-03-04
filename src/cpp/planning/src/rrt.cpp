#include "planning/joint_state.hpp"
#include "planning/rrt.hpp"
#include "planning/naive_robot_state_collection.hpp"
#include "collision_checking/fcl_robot_internal_collision_checker.hpp"
#include "transform/static_transform_tree.hpp"


namespace planning {

    RRT::RRT(Robot::Robot1* robot, PlanningRequest request): RRTBase(robot, request) {

    }

    RobotStateCollection* RRT::build_tree() {

        // Create collision checker
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

        int num_states = 1000;
        for (int i = 0; i < num_states; i++) {
            
            // generate a random state
            auto a = request;
            planning::RobotState* random_joint_state = new planning::JointState(robot->random_joint_angles());
            
            // find the neaest neighbor
            if (robot_states->size() != 0) {
                auto nearest = robot_states->nearest(random_joint_state);
                std::cout << nearest->get_configuration()[0] << std::endl;
            }


            // compute the new state


            //if (random_joint_state->is_legal(&robot, FCLRobotInternalCollisionChecker) ) {
                //      insert the new state
            robot_states->insert(random_joint_state);
            //}
        }




        return robot_states;
    }
}