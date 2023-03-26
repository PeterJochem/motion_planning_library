#ifndef RRT_
#define RRT_
#include "planning/rrt_base.hpp"

namespace planning {

class RRT: public RRTBase {

    public:
        RRT(Robot::Robot1* robot, PlanningRequest request);
        Path* solve();

    protected:
        std::map<RobotState*, RobotState*> parent_lookup;

    private:

};
}
#endif