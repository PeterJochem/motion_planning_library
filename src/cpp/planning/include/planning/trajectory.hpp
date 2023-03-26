#ifndef TRAJECTORY
#define TRAJECTORY
#include "planning/path.hpp"
#include<vector>

namespace planning {

class Trajectory {

    public:
        Trajectory(Path, std::vector<float>);
        Path get_path();
        std::vector<float> get_schedule();
    
    private:
        Path path;
        std::vector<float> schedule;
};
}
#endif