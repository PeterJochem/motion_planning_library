#include "planning/trajectory.hpp"

namespace planning {

    Trajectory::Trajectory(Path path, std::vector<float> schedule): path(path), schedule(schedule) {
    
    }

    Path Trajectory::get_path() {
        return path;
    }

    std::vector<float> Trajectory::get_schedule() {
        return schedule;
    }
}