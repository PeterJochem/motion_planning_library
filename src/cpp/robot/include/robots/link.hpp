#ifndef LINK
#define LINK
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include "meshes/mesh.hpp"
#include <string>


namespace Robot {
    
    class Link {
        public:
            Link(geometry::Transform, geometry::VisualMesh visual_mesh, geometry::CollisionMesh collision_mesh);
            geometry::Transform* get_transform();
            geometry::SymbolicTransform symbolic_transform();
            geometry::VisualMesh get_visual_mesh();
            geometry::CollisionMesh get_collision_mesh();
            std::string get_name();

        private:

            geometry::Transform transform;
            geometry::SymbolicTransform define_symbolic_transform();
            geometry::VisualMesh visual_mesh;
            geometry::CollisionMesh collision_mesh;


    };
}
#endif