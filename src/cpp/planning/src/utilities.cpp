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


    std::vector<float> subtract(std::vector<float> lhs, std::vector<float> rhs) {

        if (lhs.size() != rhs.size()) {
            throw std::runtime_error("..");
        }

        std::vector<float> difference = std::vector<float>();
        difference.reserve(lhs.size());
        for (int i = 0; i < lhs.size(); i++) {
            difference.push_back(lhs[i] - rhs[i]);
        }

        return difference;
    }

    std::vector<float> add(std::vector<float> lhs, std::vector<float> rhs) {

        if (lhs.size() != rhs.size()) {
            throw std::runtime_error("..");
        }

        std::vector<float> sum = std::vector<float>();
        sum.reserve(lhs.size());
        for (int i = 0; i < lhs.size(); i++) {
            sum.push_back(lhs[i] + rhs[i]);
        }

        return sum;
    }
    
    void multiply(std::vector<float>& configuration, float x) {

        for (int i = 0; i < configuration.size(); i++) {
            configuration[i] = configuration[i] * x;
        }

    }

    void normalize(std::vector<float>& configuration) {

        auto magnitude = 0.0;
        for (int i = 0; i < configuration.size(); i++) {
            magnitude += pow(configuration[i], 2);
        }

        magnitude = sqrt(magnitude);

        for (int i = 0; i < configuration.size(); i++) {
            configuration[i] = configuration[i] / magnitude;
        }

    }

    void scale(std::vector<float>& configuration, float magnitude) {
        
        // Normalize
        normalize(configuration);

        // Set the length of the vector to be magnitude.
        multiply(configuration, magnitude);
    }



}