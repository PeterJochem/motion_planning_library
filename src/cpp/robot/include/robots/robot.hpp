#ifndef ROBOT
#define ROBOT
#include "robots/link.hpp"
#include "robots/joint.hpp"
//#include "robots/settings.hpp"
#include "transform/transform.hpp"
#include "transform/symbolic_transform.hpp"
#include <vector>


namespace Robot {

class Robot1 {

    public:
        Robot1();
        //  virtual std::vector<Transform&> transforms_in_order() = 0;

        
        bool check_for_collision();
        bool is_current_state_in_collision();
        std::vector<float> random_joint_angles();
        // random_legal_joint_angles

        //Transform forward_kinematics(std::vector<Joint> joints);
        inline Eigen::MatrixXd jacobian(std::vector<float> angles);
        std::vector<float> inverse_kinematics(geometry::Transform);
        geometry::Transform forward_kinematics(std::vector<float>& angles);
        // jacobian();
        // inverse jacobian
        //inverse_kinematics();



    protected:
        std::vector<Link> links;
        std::vector<Joint> joints;
        std::vector<geometry::SymbolicTransform> ordered_symbolic_transforms;
        std::vector<geometry::Transform> ordered_transforms;

        

    private:
        // base_to_world_transform Transform
        // internal collision checker InternalCollisionChecker
        // external collision checker



};
}
#endif