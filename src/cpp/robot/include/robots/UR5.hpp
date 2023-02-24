#ifndef UR5
#define UR5
#include "robots/robot.hpp"
#include "robots/link.hpp"
#include "robots/joint.hpp"
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include "robots/settings.hpp"
#include "meshes/stl_parser.hpp"

#include <math.h>

namespace Robot  {
    
   class UR_5: public Robot::Robot1 {
        
        public:
            UR_5();

            std::vector<geometry::Transform> get_ordered_transforms() override;
            //std::vector<Link*> get_links() override;
            //std::vector<Joint*> get_joints() override;

        private:
            Robot::Link define_base_link();
            Robot::Link define_shoulder_link();
            Robot::Link define_upper_arm_link();
            Robot::Link define_forearm_link();
            Robot::Link define_wrist1_link();
            Robot::Link define_wrist2_link();
            Robot::Link define_wrist3_link();

            //Transform define_end_effector_transform();
            
            Robot::Joint define_base_shoulder_joint();
            Robot::Joint define_shoulder_upper_arm_joint();
            Robot::Joint define_upper_arm_forearm_joint();
            Robot::Joint define_forearm_wrist1_joint();
            Robot::Joint define_wrist1_wrist2_joint();
            Robot::Joint define_wrist2_wrist3_joint();

            std::vector<Robot::Joint> define_all_joints();
            std::vector<Robot::Link> define_all_links();

            // get_static_world_to_base_transform
            // get_end_effector_transform
            // get_transforms_in_order



   };
}
#endif