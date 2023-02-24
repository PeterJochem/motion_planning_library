#include "collision_checking/fcl_robot_internal_collision_checker.hpp"


FCLRobotInternalCollisionChecker::FCLRobotInternalCollisionChecker(Robot::Robot1& robot, geometry::TransformTree* transform_tree): robot(robot), transform_tree(transform_tree) {

    // populate fcl_models
    auto links = robot.get_links();
    for (int i = 0; i < links.size(); i++) {
        auto link = links[i];
        fcl_models[link.get_name()] = to_fcl(link);
    }
}

CollisionObjectf FCLRobotInternalCollisionChecker::create_collision_object(std::shared_ptr<Model> model, geometry::Transform transform) {

    // CollisionObjectf has a method like this: void setTransform(const Matrix3<S>& R, const Vector3<S>& T);

    Transform3f fcl_transform = conversions::to_fcl(transform, transform_tree);
    CollisionObjectf obj = CollisionObjectf(model, fcl_transform);
    
    return obj;
}

bool FCLRobotInternalCollisionChecker::check() {
    
    auto links = robot.get_links();

    /*
    auto A = *(links[links.size() - 1].get_transform());
    auto B = transform_tree->measure_transform(geometry::Frame("world"), A.get_child());
    std::cout << B << std::endl;
    */

    for (int i = 0; i < links.size(); i++) {

        auto link1_name = links[i].get_name();
        auto model1 = fcl_models[link1_name];
        auto transform1 = *(links[i].get_transform());
        auto geometry1 = create_collision_object(model1, transform1);

        for (int j = i + 1; j < links.size(); j++) {

            auto link2_name = links[j].get_name();
            auto model2 = fcl_models[link2_name];
            auto transform2 = *(links[j].get_transform());
            auto geometry2 = create_collision_object(model2, transform2);

            if (check(&geometry1, &geometry2)) {
                std::cout << link1_name << " is colliding with " << link2_name << std::endl;
                return true;
            }
        }
    }

    return false;
}

bool FCLRobotInternalCollisionChecker::check(CollisionObjectf* object1, CollisionObjectf* object2) {
    
    CollisionRequest<float> request;
    CollisionResult<float> result;
    collide(object1, object2, request, result);
    return result.isCollision();
}

std::shared_ptr<Model> FCLRobotInternalCollisionChecker::to_fcl(Robot::Link link) {

    auto collision_mesh = link.get_collision_mesh();
    auto vertices = conversions::to_fcl(collision_mesh.get_vertices());
    auto triangles = conversions::to_fcl(collision_mesh.get_triangles());

    std::shared_ptr<Model> geom = std::make_shared<Model>();
    geom->beginModel();
    geom->addSubModel(vertices, triangles);
    geom->endModel();
    
    return geom;
}