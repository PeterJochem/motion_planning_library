#include "planning/utilities.hpp"

namespace planning {

     float sum_square_errors(RobotState* state1, RobotState* state2) {

        if (!state1 || !state2) {
            // throw exception
        }
        else if (state1->dimension() != state2->dimension()) {
            // throw exception
        }

        float error = 0.;
        auto configuration1 = state1->get_configuration();
        auto configuration2 = state2->get_configuration();
        for (int i = 0; i < configuration1.size(); i++) {
            error += std::pow((configuration1[i] - configuration2[i]), 2);
        }

        return error;
    }

}