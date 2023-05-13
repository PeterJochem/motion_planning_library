#ifndef RRT_STAR
#define RRT_STAR
#include "planning/rrt_base.hpp"
#include "planning/rrt_planning_request.hpp"



namespace planning {

    class RRTStar: public RRTBase {

        public:
            RRTStar(Robot::Robot1* robot, RRTPlanningRequest* request);
            Path* solve();

        protected:
            std::map<RobotState*, RobotState*> parent_lookup;

        private:
            RRTPlanningRequest* request;
    };
}


#endif