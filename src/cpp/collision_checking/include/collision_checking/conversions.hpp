#include "primitives/point.hpp"
#include "transform/transform_tree.hpp"
#include "transform/utilities.hpp"
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"
#include <vector>

namespace conversions {

    std::vector<fcl::Vector3f> to_fcl(std::vector<geometry::Point3D>);
    std::vector<fcl::Triangle> to_fcl(std::vector<std::tuple<int, int, int>>);
    
    fcl::Vector3f to_fcl(geometry::Point3D);
    fcl::Triangle to_fcl(std::tuple<int, int, int>);

    fcl::Transform3f to_fcl(geometry::Transform, geometry::TransformTree*);


}