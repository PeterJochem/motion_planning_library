#ifndef RRT_BASE
#define RRT_BASE
#include "robots/robot.hpp"
#include "planning/robot_state_collection.hpp"
#include "planning/path.hpp"
#include "planning/planning_request.hpp"
#include "planning/joint_state.hpp"
#include "transform/static_transform_tree.hpp"
#include "planning/naive_robot_state_collection.hpp"



namespace planning {

    class RRTBase {

        public:
            RRTBase();
            RRTBase(Robot::Robot1*);
        
        protected:
            Path* solve();
            Robot::Robot1* robot;
            geometry::StaticTransformTree* transform_tree;
            FCLRobotInternalCollisionChecker* internal_collision_checker;
            RobotStateCollection* robot_states;

        private:
    };
}
#endif