#include "collision_checking/fcl_robot_internal_collision_checker.hpp"


FCLRobotInternalCollisionChecker::FCLRobotInternalCollisionChecker(Robot::Robot1& robot, geometry::TransformTree* transform_tree): robot(robot), transform_tree(transform_tree) {

    // populate fcl_models
    auto links = robot.get_links();
    for (int i = 0; i < links.size(); i++) {
        auto link = links[i];
        auto key = link.get_name();
        auto model = to_fcl(link);
        auto collision_object = create_collision_object(model, geometry::Transform::identity(geometry::Frame("world"), geometry::Frame("world")));
        fcl_models[key] = model;
        fcl_collision_objects[key] = collision_object;
    }
}

CollisionObjectf* FCLRobotInternalCollisionChecker::create_collision_object(std::shared_ptr<Model> model, geometry::Transform transform) {

    Transform3f fcl_transform = conversions::to_fcl(transform, transform_tree);
    return new CollisionObjectf(model, fcl_transform);    
}

bool FCLRobotInternalCollisionChecker::check() {
    
    auto links = robot.get_links();

    // Update all the transforms for each collision object.
    for (int i = 0; i < links.size(); i++) {

        auto link_name = links[i].get_name();
        auto transform = *(links[i].get_transform());
        auto geometry = fcl_collision_objects[link_name];
        geometry->setTransform(conversions::to_fcl(transform, transform_tree));
    }

    // Check each pair of links for collisions.
    for (int i = 0; i < links.size(); i++) {

        auto link1_name = links[i].get_name();
        auto transform1 = *(links[i].get_transform());
        auto geometry1 = fcl_collision_objects[link1_name];

        for (int j = i + 1; j < links.size(); j++) {

            auto link2_name = links[j].get_name();
            auto transform2 = *(links[j].get_transform());
            auto geometry2 = fcl_collision_objects[link2_name];

            if (check(geometry1, geometry2)) {
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