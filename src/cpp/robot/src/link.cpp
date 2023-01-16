#include "robots/link.hpp"


namespace Robot {


    Link::Link(geometry::Transform transform, std::string visual_mesh_file_name, std::string collision_mesh_file_name): transform(transform) {
    }

    geometry::Transform Link::get_transform() {
        return transform;
    }

    geometry::SymbolicTransform Link::symbolic_transform() {
        
        double x = transform.get_x();
        double y = transform.get_y();
        double z = transform.get_z();
        double roll = transform.get_roll();
        double pitch = transform.get_pitch();
        double yaw = transform.get_yaw();
        return geometry::SymbolicTransform(x, y, z, roll, pitch, yaw);
    }

}