#include "collision_checking/conversions.hpp"

namespace conversions {

    std::vector<fcl::Vector3f> to_fcl(std::vector<geometry::Point3D> points) {
        
        std::vector<fcl::Vector3f> vectors;
        vectors.reserve(points.size());
        for (std::vector<geometry::Point3D>::iterator itr = points.begin(); itr != points.end(); itr++) {
            vectors.push_back(to_fcl(*itr));
        }

        return vectors;
    }

    fcl::Vector3f to_fcl(geometry::Point3D point) {
        return fcl::Vector3f(point.get_x(), point.get_y(), point.get_z());
    }

    fcl::Triangle to_fcl(std::tuple<int, int, int> indices) {
        std::size_t index1 = std::get<0>(indices);
        std::size_t index2 = std::get<1>(indices); 
        std::size_t index3 = std::get<2>(indices); 
        return fcl::Triangle(index1, index2, index3);
    }

    std::vector<fcl::Triangle> to_fcl(std::vector<std::tuple<int, int, int>> indices) {

        std::vector<fcl::Triangle> triangles;
        triangles.reserve(indices.size());
        for (std::vector<std::tuple<int, int, int>>::iterator itr = indices.begin(); itr != indices.end(); itr++) {
            triangles.push_back(to_fcl(*itr));
        }

        return triangles;
    }

    fcl::Transform3f to_fcl(geometry::Transform transform, geometry::TransformTree* tree) {

        using namespace fcl;
        Matrix3f R;
        
        Transform3f pose = Transform3f::Identity();
        
        auto transform_in_world = tree->measure_transform(geometry::Frame("world"), transform.get_child());

        auto position = transform_in_world.getPosition().coordinates();
        auto euler_angles = transform_in_world.getEulerAngles().coordinates();

        Vector3f T(position[0], position[1], position[2]);

        float roll, pitch, yaw; 
        roll = euler_angles[0];
        pitch = euler_angles[1];
        yaw = euler_angles[2];
        auto A = utilities::euler_angles_to_rotation_matrix(roll, pitch, yaw);

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R(i, j) = A(i, j);
            }
        }

        pose.linear() = R;
        pose.translation() = T;
   
        return pose;    
    }

    std::shared_ptr<Model> to_fcl(Robot::Link link) {

        auto collision_mesh = link.get_collision_mesh();
        auto vertices = conversions::to_fcl(collision_mesh.get_vertices());
        auto triangles = conversions::to_fcl(collision_mesh.get_triangles());

        std::shared_ptr<Model> geom = std::make_shared<Model>();
        geom->beginModel();
        geom->addSubModel(vertices, triangles);
        geom->endModel();
        
        return geom;
    }
}