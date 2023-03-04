#include "robots/utilities.hpp"

namespace utilities {

    double random_in_interval(double M, double N) {
        return M + (rand() / ( RAND_MAX / (N-M) ) ) ;  
    }
}