#include "robots/robot.hpp"

namespace Robot {


    Robot1::Robot1() {
        ;
    }

    geometry::Transform Robot1::forward_kinematics(std::vector<float> angles) {


        geometry::Frame parent = ordered_transforms[0].get_parent();
        geometry::Frame child = ordered_transforms[ordered_transforms.size() - 1].get_child();

        geometry::SymbolicTransform result = ordered_symbolic_transforms[0];
        for (int i = 1; i < ordered_symbolic_transforms.size(); i++) {
            result = result * ordered_symbolic_transforms[i];
        }

        GiNaC::exmap map;
        for (int i = 0; i < joints.size(); i++) {
            
            GiNaC::symbol joint_angle(joints[i].name());
            map[joint_angle] = 1.57;
        }

        auto result2 = GiNaC::evalf(result.matrix.subs(map));

        // convert to a non symbolic Transform;


        return geometry::Transform::identity(parent, child);
    }

}