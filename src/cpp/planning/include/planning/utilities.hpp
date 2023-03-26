#include "planning/robot_state.hpp"
#include <math.h>
#include <numeric>

namespace planning {

    float sum_square_errors(RobotState* state1, RobotState* state2);
    std::vector<float> subtract(std::vector<float> configuration1, std::vector<float> configuration2);
    std::vector<float> add(std::vector<float> lhs, std::vector<float> rhs);
    void scale(std::vector<float>& configuration, float magnitude);

}