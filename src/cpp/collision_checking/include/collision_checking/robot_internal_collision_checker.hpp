#ifndef ROBOT_INTERNAL_COLLISION_CHECKER
#define ROBOT_INTERNAL_COLLISION_CHECKER

#include "collision_checking/collision_checker.hpp"
#include "robots/robot.hpp"


class RobotInternalCollisionChecker : public CollisionChecker {

    public:
        RobotInternalCollisionChecker();
        virtual bool check() = 0;

    private:
};
#endif