#include "robots/joint.hpp"


namespace Robot {

    Joint::Joint(geometry::Transform transform, geometry::Axis axis): transform(transform), axis(axis) {
        zero_angle_transform = transform;

    }

    geometry::Transform Joint::get_transform() {
        return transform;
    }

    void Joint::apply_rotation(float radians) {

        if (axis.is_pure_roll()) {

            float new_roll = zero_angle_transform.get_roll() + radians;
            transform.set_roll(new_roll);
        }
        else if (axis.is_pure_pitch()) {

            float new_pitch = zero_angle_transform.get_pitch() + radians;
            transform.set_pitch(new_pitch);
        }
        else if (axis.is_pure_yaw()) {

            float new_yaw = zero_angle_transform.get_yaw() + radians;
            transform.set_yaw(new_yaw);
        }
        else {
            throw std::runtime_error("The axis must be a pure roll, pitch, or yaw.");
        }

        angle = radians;
    }

    std::string Joint::name() {

        return transform.get_parent().get_name() + "->" + transform.get_child().get_name();
    }

    geometry::SymbolicTransform Joint::symbolic_transform() {

        double x = transform.get_x();
        double y = transform.get_y();
        double z = transform.get_z();

        if (axis.is_pure_roll()) {

            GiNaC::symbol roll(name());
            return geometry::SymbolicTransform(x, y, z, roll, transform.get_pitch(), transform.get_yaw());
        }
        else if (axis.is_pure_pitch()) {

            GiNaC::symbol pitch(name());
            return geometry::SymbolicTransform(x, y, z, transform.get_roll(), pitch, transform.get_yaw());
        }
        else if (axis.is_pure_yaw()) {

            GiNaC::symbol yaw(name());
            return geometry::SymbolicTransform(x, y, z, transform.get_roll(), transform.get_pitch(), yaw);
        }
        

        throw std::runtime_error("The axis must be a pure roll, pitch, or yaw.");
    }

}