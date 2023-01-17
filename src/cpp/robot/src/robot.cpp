#include "robots/robot.hpp"
#include "conversions/conversions.hpp"
#include <cassert>

namespace Robot {


    Robot1::Robot1() {
        ;
    }

    
    geometry::Transform Robot1::forward_kinematics(std::vector<float>& angles) {

        geometry::Frame parent = ordered_transforms[0].get_parent();
        geometry::Frame child = ordered_transforms[ordered_transforms.size() - 1].get_child();

        
        geometry::SymbolicTransform result = ordered_symbolic_transforms[0];
        for (int i = 1; i < ordered_symbolic_transforms.size(); i++) {
            result = result * ordered_symbolic_transforms[i];
        }

        GiNaC::exmap map;
        for (int i = 0; i < joints.size(); i++) {            
            map[joints[i].joint_symbol] = angles[i];
        }

        auto result2 = GiNaC::evalf(result.matrix.subs(map));
        return geometry::conversions::convert_to_transform(parent, child, result2);
    }

    Eigen::MatrixXd Robot1::jacobian(std::vector<float> current_angles) {

        geometry::Transform current_transform = forward_kinematics(current_angles);
        float delta = 0.0012;
        Eigen::MatrixXd result(6, 6);

        // Compute the derivatives of x.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] = current_angles[i] + delta;
            float nudged_value = forward_kinematics(current_angles).get_x();
            float dx_derivative = (current_transform.get_x() - nudged_value) / delta;

            result(i, 0) = dx_derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of y.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_y();
            float dy_derivative = (current_transform.get_y() - nudged_value)/delta;

            result(i, 1) = dy_derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of z.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_z();
            float dz_derivative = (current_transform.get_z() - nudged_value)/delta;

            result(i, 2) = dz_derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        // Compute the derivatives of roll.
        for (int i = 0; i < joints.size(); i++) {
            
            current_angles[i] = current_angles[i] + delta;
            float nudged_value = forward_kinematics(current_angles).get_roll();
            float droll_derivative = (current_transform.get_roll() - nudged_value)/delta;

            result(i, 3) = droll_derivative + 0.0001;
            
            current_angles[i] = current_angles[i] - delta; // Undo the addition above.
        }

        // Compute the derivatives of pitch.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_pitch();
            float dpitch_derivative = (current_transform.get_pitch() - nudged_value)/delta;

            result(i, 4) = dpitch_derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }
        
        // Compute the derivatives of yaw.
        for (int i = 0; i < joints.size(); i++) {

            current_angles[i] += delta;
            float nudged_value = forward_kinematics(current_angles).get_yaw();
            float dyaw_derivative = (current_transform.get_yaw() - nudged_value)/delta;

            result(i, 5) = dyaw_derivative;

            current_angles[i] -= delta; // Undo the addition above.
        }

        return result;
    }

    std::vector<float> Robot1::inverse_kinematics(geometry::Transform goal) {
                
        float tolerance = 0.001;
        float max_iteration = 1000;
        float error = 10000000.0;
        std::vector<float> current_angles =  {-1.5, 2, 1.3, -0.4, 1.4, 0.4};
        geometry::Transform current_transform = forward_kinematics(current_angles);

        int iteration = 0;
        while (error > tolerance && iteration < max_iteration) {

            auto jacobian_matrix = jacobian(current_angles);

            auto inverse_jacobian = jacobian_matrix.transpose().inverse();
            //((jacobian_matrix.transpose() * jacobian_matrix).inverse()) * jacobian_matrix.transpose();
           
            Eigen::MatrixXd goal_pose(6, 1);
            Eigen::MatrixXd current_pose(6, 1);
            
            auto[x, y, z, roll, pitch, yaw] = utilities::to_pose_vector(goal.matrix);
            goal_pose << x, y, z, roll, pitch, yaw;
            
            auto[x2, y2, z2, roll2, pitch2, yaw2] = utilities::to_pose_vector(current_transform.matrix);
            current_pose << x2, y2, z2, roll2, pitch2, yaw2;

            auto delta_transform = goal_pose - current_pose;
            auto delta_theta = inverse_jacobian * delta_transform;

            for (int i = 0; i < current_angles.size(); i++) {
                current_angles[i] -= (delta_theta(i, 0)) * 1.; //0.05;
            }

            current_transform = forward_kinematics(current_angles);
            error = utilities::sum_square_error(current_transform.matrix, goal.matrix);
            iteration += 1;
            std::cout << "Error: " << error << std::endl;
        }

        return current_angles;
    }
}