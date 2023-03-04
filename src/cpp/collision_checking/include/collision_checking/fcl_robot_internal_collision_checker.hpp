#ifndef FCL_ROBOT_INTERNAL_COLLISION_CHECKER
#define FCL_ROBOT_INTERNAL_COLLISION_CHECKER

#include "collision_checking/robot_internal_collision_checker.hpp"
#include "collision_checking/conversions.hpp"
#include "transform/transform_tree.hpp"
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/config.h"
#include <map>
#include <string>

using namespace fcl;

typedef BVHModel<OBBRSSf> Model;

class FCLRobotInternalCollisionChecker: public RobotInternalCollisionChecker {

    public:
        FCLRobotInternalCollisionChecker(Robot::Robot1&, geometry::TransformTree*);
        bool check();

    private:
        bool check(Model&, geometry::Transform&, Model&, geometry::Transform&);
        bool check(CollisionObjectf*, CollisionObjectf*);
        CollisionObjectf* create_collision_object(std::shared_ptr<Model>, geometry::Transform);
        std::map<std::string, std::shared_ptr<Model>> fcl_models;
        std::map<std::string, CollisionObjectf*> fcl_collision_objects;
        std::shared_ptr<Model> to_fcl(Robot::Link);
        Robot::Robot1& robot;
        geometry::TransformTree* transform_tree;
        
};
#endif