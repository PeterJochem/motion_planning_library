#include "robots/robot.hpp"
#include "robots/utilities.hpp"
#include "conversions/conversions.hpp"
#include <cassert>

namespace Robot {


    Robot1::Robot1() {
    }

    std::vector<float> Robot1::random_joint_angles() {

        auto random_angles = std::vector<float>();
        for (std::vector<Robot::Joint>::iterator itr = joints.begin(); itr != joints.end(); itr++) {
            random_angles.push_back(utilities::random_in_interval(-1, 1));
        }

        return random_angles;
    }

    
    geometry::Transform Robot1::forward_kinematics(std::vector<float>& angles) {

        geometry::Frame parent = ordered_transforms[0]->get_parent();
        geometry::Frame child = ordered_transforms[ordered_transforms.size() - 1]->get_child();

        geometry::SymbolicTransform product = ordered_symbolic_transforms[0];
        for (int i = 1; i < ordered_symbolic_transforms.size(); i++) {
            product = product * ordered_symbolic_transforms[i];
        }

        GiNaC::exmap map;
        for (int i = 0; i < joints.size(); i++) {            
            map[joints[i].joint_symbol] = angles[i];
        }

        auto result = GiNaC::evalf(product.matrix.subs(map));
        return geometry::conversions::convert_to_transform(parent, child, result);
    }

    void Robot1::set_joint_angles(std::vector<float> angles) {

        // Error Check here
        for (int i = 0; i < angles.size(); i++) {
            joints[i].apply_rotation(angles[i]);
        }

    }

    Eigen::MatrixXd Robot1::jacobian(std::vector<float> current_angles) {

        geometry::Transform current_transform = forward_kinematics(current_angles);
        float delta = 0.0012;
        Eigen::MatrixXd result(6, joints.size());

        // Compute the derivatives of x.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] = current_angles[i] + delta;
            float nudged_value = forward_kinematics(current_angles).get_x();
            float derivative = (current_transform.get_x() - nudged_value) / delta;

            result(i, 0) = derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of y.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_y();
            float derivative = (current_transform.get_y() - nudged_value)/delta;

            result(i, 1) = derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of z.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_z();
            float derivative = (current_transform.get_z() - nudged_value)/delta;

            result(i, 2) = derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of roll.
        for (int i = 0; i < joints.size(); i++) {
            
            current_angles[i] = current_angles[i] + delta;
            float nudged_value = forward_kinematics(current_angles).get_roll();
            float derivative = (current_transform.get_roll() - nudged_value)/delta;

            result(i, 3) = derivative + 0.0001;
            
            current_angles[i] = current_angles[i] - delta; // Undo the addition above.
        }

        // Compute the derivatives of pitch.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_pitch();
            float derivative = (current_transform.get_pitch() - nudged_value)/delta;

            result(i, 4) = derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }
        
        // Compute the derivatives of yaw.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_yaw();
            float derivative = (current_transform.get_yaw() - nudged_value)/delta;

            result(i, 5) = derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        return result;
    }

    /** Finds a set of joint angles which make the robot's end effector have the given trnsform. 
     *  Implemnts the Newton-Raphson method for non-linear root finding as described in the link below.
     *      hades.mech.northwestern.edu/images/7/7f/MR.pdf
    **/
    std::vector<float>  Robot1::inverse_kinematics(geometry::Transform goal, std::vector<float> joint_angles, int max_iterations, float tolerance) {
        
        int iteration = 0;
        auto current_transform = forward_kinematics(joint_angles);
        float error = utilities::sum_square_error(current_transform.matrix, goal.matrix);

        while (error > tolerance && iteration < max_iterations) {

            auto jacobian_matrix = jacobian(joint_angles);

            auto inverse_jacobian = jacobian_matrix.transpose().inverse();
            //Pseucoinverse - ((jacobian_matrix.transpose() * jacobian_matrix).inverse()) * jacobian_matrix.transpose();
           
            Eigen::MatrixXd goal_pose(6, 1);
            Eigen::MatrixXd current_pose(6, 1);
            
            auto[x, y, z, roll, pitch, yaw] = utilities::to_pose_vector(goal.matrix);
            goal_pose << x, y, z, roll, pitch, yaw;
            
            auto[x2, y2, z2, roll2, pitch2, yaw2] = utilities::to_pose_vector(current_transform.matrix);
            current_pose << x2, y2, z2, roll2, pitch2, yaw2;

            auto delta_theta = inverse_jacobian * (goal_pose - current_pose);

            for (int i = 0; i < joint_angles.size(); i++) {
                joint_angles[i] -= (delta_theta(i, 0)) * 1.;
            }

            current_transform = forward_kinematics(joint_angles);
            error = utilities::sum_square_error(current_transform.matrix, goal.matrix);
            iteration += 1;
            std::cout << "Error: " << error << std::endl;
        }

        return joint_angles;
    }

    std::vector<float> Robot1::inverse_kinematics(geometry::Transform goal) {
                
        float tolerance = 0.001;
        float max_iterations = 1000;
        std::vector<float> start_angles =  {-1.5, 2, 1.3, -0.4, 1.4, 0.4};
        return inverse_kinematics(goal, start_angles, max_iterations, tolerance);
    }

    std::vector<Robot::Link> Robot1::get_links() {
        return links;
    }

    
}