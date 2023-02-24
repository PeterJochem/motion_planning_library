#include "collision_checking/collision_checker.hpp"
#include "robots/robot.hpp"


class RobotInternalCollisionChecker : public CollisionChecker {

    public:
        RobotInternalCollisionChecker();
        bool check();

    private:


};