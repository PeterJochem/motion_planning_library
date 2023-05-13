#ifndef NAIVE_TIME_SCALER
#define NAIVE_TIME_SCALER
#include "planning/time_scaler.hpp"


namespace planning {

    class NaiveTimeScaler : public TimeScaler{

        public:
            NaiveTimeScaler(Path path);
            Trajectory scale();
    };
}
#endif