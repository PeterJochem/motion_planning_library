#include "planning/rrt_base.hpp"

namespace planning {

    RRTBase::RRTBase(Robot::Robot1* robot): robot(robot) {
        
        using namespace geometry;
        Frame root = Frame("world");
        transform_tree = new StaticTransformTree(root);

        auto transforms = robot->get_ordered_transforms();
        for (int i = 0; i < transforms.size(); i++) {
            transform_tree->add(transforms[i]);
        }

        internal_collision_checker = new FCLRobotInternalCollisionChecker(*robot, transform_tree);
        robot_states = new NaiveRobotStateCollection();
    }

}