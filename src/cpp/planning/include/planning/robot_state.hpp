#include <vector>

namespace planning {

    class RobotState {

        public:
            RobotState();
            virtual std::vector<float> configuration() = 0;
        private:

        protected:

    };

}