#ifndef RRT_BASE
#define RRT_BASE
#include "robots/robot.hpp"
#include "planning/robot_state_collection.hpp"
#include "planning/path.hpp"
#include "planning/planning_request.hpp"
#include "planning/joint_state.hpp"


namespace planning {

class RRTBase {

    public:
        RRTBase();
        RRTBase(Robot::Robot1*, PlanningRequest);
    
    protected:
        Path* solve();
        PlanningRequest request;
        Robot::Robot1* robot;

    private:


};
}
#endif