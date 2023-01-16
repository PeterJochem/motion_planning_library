#ifndef LINK
#define LINK
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"

namespace Robot {
    
    class Link {
        public:
            Link(geometry::Transform, std::string visual_mesh_file_name, std::string collision_mesh_file_name);
            geometry::Transform get_transform();
            geometry::SymbolicTransform symbolic_transform();

        private:

            geometry::Transform transform;
            geometry::SymbolicTransform define_symbolic_transform();
            //Mesh collision_mesh;
            //Mesh visual_mesh;



    };
}
#endif