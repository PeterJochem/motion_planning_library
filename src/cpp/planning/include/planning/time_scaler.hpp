#ifndef TIME_SCALER
#define TIME_SCALER
#include "planning/path.hpp"
#include "planning/trajectory.hpp"
#include<vector>

namespace planning {

    class TimeScaler {

        public:
            TimeScaler(Path);
            virtual Trajectory scale() = 0;
        
        protected:
            Path path;
    };
}
#endif