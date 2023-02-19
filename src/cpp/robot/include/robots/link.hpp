#ifndef LINK
#define LINK
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include "meshes/mesh.hpp"


namespace Robot {
    
    class Link {
        public:
            Link(geometry::Transform, geometry::VisualMesh visual_mesh, std::string collision_mesh_file_name);
            geometry::Transform* get_transform();
            geometry::SymbolicTransform symbolic_transform();
            geometry::VisualMesh get_visual_mesh();

        private:

            geometry::Transform transform;
            geometry::SymbolicTransform define_symbolic_transform();
            //geometry::Mesh collision_mesh;
            geometry::VisualMesh visual_mesh;



    };
}
#endif