#include "robots/robot.hpp"
#include <vector>

namespace planning {

    class RobotState {

        public:
            RobotState();
            virtual std::vector<float> get_configuration() = 0;
        private:

        protected:

    };

}