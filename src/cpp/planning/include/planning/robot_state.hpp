#ifndef ROBOT_STATE
#define ROBOT_STATE
#include "robots/robot.hpp"
#include <vector>

namespace planning {

    class RobotState {

        public:
            RobotState();
            virtual std::vector<float> get_configuration() = 0;
            virtual int dimension() = 0;
            virtual float distance(RobotState*) = 0;
        private:

        protected:

    };

}
#endif