#ifndef ROBOT
#define ROBOT
#include "robots/link.hpp"
#include "robots/joint.hpp"
//#include "robots/settings.hpp"
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include "meshes/mesh.hpp"
#include <vector>


namespace Robot {

class Robot1 {

    public:
        Robot1();
        //  virtual std::vector<Transform&> transforms_in_order() = 0;

        
        bool check_for_collision(std::vector<float>);
        bool is_current_state_in_collision();
        std::vector<float> random_joint_angles();
        // random_legal_joint_angles

        inline Eigen::MatrixXd jacobian(std::vector<float> angles);
        std::vector<float> inverse_kinematics(geometry::Transform);
        geometry::Transform forward_kinematics(std::vector<float>& angles);
        
        virtual std::vector<geometry::Transform> get_ordered_transforms() = 0;
        //virtual std::vector<Link*> get_links() = 0;
        //virtual std::vector<Joint*> get_joints() = 0;


        void set_joint_angles(std::vector<float> angles);
        std::vector<Robot::Link> get_links();


    protected:
        std::vector<Link> links;
        std::vector<Joint> joints;
        std::vector<geometry::SymbolicTransform> ordered_symbolic_transforms;
        std::vector<geometry::Transform*> ordered_transforms;
        std::vector<float> inverse_kinematics(geometry::Transform, std::vector<float> joint_angles, int max_iterations, float tolerance);

    private:
        // base_to_world_transform Transform
        // internal collision checker InternalCollisionChecker
        // external collision checker



};
}
#endif