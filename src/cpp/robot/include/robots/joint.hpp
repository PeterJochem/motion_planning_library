#ifndef JOINT
#define JOINT
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include "primitives/axis.hpp"
#include <string>


namespace Robot {

    class Joint {

        public:
            Joint(geometry::Transform, geometry::Axis);
            geometry::Transform get_transform();
            void apply_rotation(float radians);
            geometry::SymbolicTransform symbolic_transform();
            std::string name();
            GiNaC::symbol joint_symbol;
            

        private:
            geometry::SymbolicTransform define_symbolic_transform();
            geometry::Transform zero_angle_transform;
            geometry::Transform transform;
            geometry::Axis axis;
            float angle;
    
    };
}
#endif