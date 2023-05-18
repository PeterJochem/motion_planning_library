#ifndef RRT_CONNECT
#define RRT_CONNECT
#include "planning/rrt_base.hpp"
#include "planning/rrt_planning_request.hpp"
#include <bits/stdc++.h>


namespace planning {

    class RRTConnect: public RRTBase {

        public:
            RRTConnect(Robot::Robot1* robot, RRTPlanningRequest* request);
            Path* solve();

        protected:
            std::map<RobotState*, RobotState*> parent_lookup;

        private:
            RRTPlanningRequest* request;      

    };
}

#endif