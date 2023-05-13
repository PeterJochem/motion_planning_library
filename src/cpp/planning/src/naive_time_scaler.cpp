#include "planning/naive_time_scaler.hpp"


namespace planning {

    NaiveTimeScaler::NaiveTimeScaler(Path path): TimeScaler(path) {

    }

    Trajectory NaiveTimeScaler::scale() {

        std::vector<float> schedule = std::vector<float>();
        for (int i = 0; i < path.size(); i++){
            schedule.push_back(i);
        }

        return planning::Trajectory(path, schedule);
    }
}
