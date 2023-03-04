#ifndef RRT_
#define RRT_
#include "planning/rrt_base.hpp"

namespace planning {

class RRT: public RRTBase {

    public:
        RRT(Robot::Robot1* robot, PlanningRequest request);
        RobotStateCollection* build_tree();

    protected:


    private:

};
}
#endif