#include "robots/UR5.hpp"

namespace Robot {


    Robot::UR_5::UR_5() {

        this->links = define_all_links();
        this->joints = define_all_joints();

        // Make a function
        // Traverse the links and joints and make an ordered list of Transforms
        for (int i = 0; i < joints.size(); i++) {
            ordered_symbolic_transforms.push_back(  links[i].symbolic_transform());
            ordered_symbolic_transforms.push_back(  joints[i].symbolic_transform()); 
            //
            ordered_transforms.push_back(  links[i].get_transform());
            ordered_transforms.push_back(  joints[i].get_transform());
        
        }
        ordered_symbolic_transforms.push_back( links[links.size() - 1].symbolic_transform()  );
        ordered_transforms.push_back(  links[links.size() - 1].get_transform());
        // End of function
        //

    }

    
    Robot::Link UR_5::define_base_link() {

        std::string link_name = "base";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform root_to_base = geometry::Transform(ROOT_FRAME_NAME, link_name, 1., 0., 0., 0., 0., 0.);
        return Robot::Link(root_to_base, visual_mesh_file, collision_mesh_file);
    }

    Robot::Link UR_5::define_shoulder_link() {


        std::string link_name = "shoulder";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("base_shoulder_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }
    
    Robot::Link UR_5::define_upper_arm_link() {

        std::string link_name = "upperarm";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("shoulder_upper_arm_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }

    Robot::Link UR_5::define_forearm_link() {

        std::string link_name = "forearm";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("upper_arm_forearm_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }

    
    Robot::Link UR_5::define_wrist1_link() {

        std::string link_name = "wrist1";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("forearm_wrist1_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }

    
    Robot::Link UR_5::define_wrist2_link() {

        std::string link_name = "wrist2";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("wrist1_wrist2_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }

    
    Robot::Link UR_5::define_wrist3_link() {

        std::string link_name = "wrist3";
        std::string visual_mesh_file = "";
        std::string collision_mesh_file = "";

        geometry::Transform transform = geometry::Transform("wrist2_wrist3_joint", link_name, 0., 0., 0., 0., 0., 0.);
        return Link(transform, visual_mesh_file, collision_mesh_file);
    }

    Robot::Joint UR_5::define_base_shoulder_joint() {

        geometry::Transform transform = geometry::Transform("base_link", "base_shoulder_joint", 0., 0., 0.089159, 0., 0., 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 0, 1));
        return Joint(transform, axis);
    }

    Robot::Joint UR_5::define_shoulder_upper_arm_joint() {

        geometry::Transform transform = geometry::Transform("shoulder_link", "shoulder_upper_arm_joint", 0., 0.13585, 0., 0., M_PI/2, 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 1, 0));
        return Joint(transform, axis);
    }
    
    Robot::Joint UR_5::define_upper_arm_forearm_joint() {

        geometry::Transform transform = geometry::Transform("upper_arm_link", "upper_arm_forearm_joint", 0., -0.1197 + -0.001, 0.42500, 0., 0., 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 1, 0));
        return Joint(transform, axis);
    }
    
    
    Robot::Joint UR_5::define_forearm_wrist1_joint() {

        geometry::Transform transform = geometry::Transform("forearm_link", "forearm_wrist1_joint", 0., 0.001, 0.39225, 0., M_PI/2, 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 1, 0));
        return Joint(transform, axis);
    }
    
    
    Robot::Joint UR_5::define_wrist1_wrist2_joint() {

        geometry::Transform transform = geometry::Transform("wrist1_link", "wrist1_wrist2_joint", 0., 0.093, 0., 0., 0., 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 0, 1));
        return Joint(transform, axis);
    }
    
    Robot::Joint UR_5::define_wrist2_wrist3_joint() {

        geometry::Transform transform = geometry::Transform("wrist2_link", "wrist2_wrist3_joint", 0., 0.001, 0.09465, 0., 0., 0.);
        geometry::Axis axis = geometry::Axis(geometry::Vector3D(0, 1, 0));
        return Joint(transform, axis);
    }

    std::vector<Joint> UR_5::define_all_joints() {

        return {define_base_shoulder_joint(), define_shoulder_upper_arm_joint(), define_upper_arm_forearm_joint(), define_forearm_wrist1_joint(), define_wrist1_wrist2_joint(), define_wrist2_wrist3_joint() };
    }
    
    
    std::vector<Link> UR_5::define_all_links() {

        return {define_base_link(), define_shoulder_link(), define_upper_arm_link(), define_forearm_link(), define_wrist1_link(), define_wrist2_link(), define_wrist3_link()};
    }
    

}