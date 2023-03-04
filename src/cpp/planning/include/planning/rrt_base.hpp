#ifndef RRT_BASE
#define RRT_BASE
#include "robots/robot.hpp"
#include "planning/robot_state_collection.hpp"
#include "planning/path.hpp"
#include "planning/planning_request.hpp"

namespace planning {

class RRTBase {

    public:
        RRTBase();
        RRTBase(Robot::Robot1*, PlanningRequest);
    
    protected:
        RobotStateCollection* build_tree();
        Path convert_tree_to_path(RobotStateCollection*);
        PlanningRequest request;
        Robot::Robot1* robot;

    private:


};
}
#endif