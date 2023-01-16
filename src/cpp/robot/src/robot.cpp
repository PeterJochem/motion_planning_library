#include "robots/robot.hpp"
#include "conversions/conversions.hpp"

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
            
            //GiNaC::symbol joint_angle(joints[i].name());
            map[joints[i].joint_symbol ] = angles[i];
        }

        auto result2 = GiNaC::evalf(result.matrix.subs(map));

        //std::cout << result2 << std::endl;

        return geometry::conversions::convert_to_transform(parent, child, result2);
    }

}