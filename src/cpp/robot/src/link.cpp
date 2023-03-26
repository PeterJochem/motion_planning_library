#include "robots/link.hpp"

namespace Robot {


    Link::Link(geometry::Transform transform, geometry::VisualMesh visual_mesh, geometry::CollisionMesh collision_mesh): transform(transform), visual_mesh(visual_mesh), collision_mesh(collision_mesh) {

    }

    geometry::Transform* Link::get_transform() {
        return &transform;
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

    geometry::VisualMesh Link::get_visual_mesh() {
        return visual_mesh;
    }

    std::string Link::get_name() {
        return transform.get_child().get_name();
    }

    geometry::CollisionMesh Link::get_collision_mesh() {
        return collision_mesh;
    }

}