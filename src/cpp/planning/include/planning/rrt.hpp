#ifndef RRT_
#define RRT_
#include "planning/rrt_base.hpp"
#include "planning/rrt_planning_request.hpp"


namespace planning {

class RRT: public RRTBase {

    public:
        RRT(Robot::Robot1* robot, RRTPlanningRequest* request);
        Path* solve();

    protected:
        std::map<RobotState*, RobotState*> parent_lookup;

    private:
        RRTPlanningRequest* request;      
};
}
#endif