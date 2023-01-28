#include <gtest/gtest.h>
#include "robots/UR5.hpp"
#include "transform/transform.hpp"
#include <chrono>


TEST(UR5_5_Suite, test_forward_kinematics1) {

    Robot::UR_5 robot = Robot::UR_5();
    std::vector<float> joint_angles = {0., 0., 0., 0., 0., 0.};
    auto transform = robot.forward_kinematics(joint_angles);

    Eigen::Matrix4d expected;
    expected << -1., 0., 0., 1.81725, 0., -0.707107,  0.707107, 0.11015, 0.,  0.707107, 0.707107, -0.005491, 0., 0., 0., 1.;
    auto expected_transform = geometry::Transform(transform.get_parent(), transform.get_child(), expected);

    EXPECT_EQ(transform, expected_transform);
}

TEST(UR5_5_Suite, test_forward_kinematics2) {

    Robot::UR_5 robot = Robot::UR_5();
    std::vector<float> joint_angles = {1., 2., 0., 0., 0.5, 2.5};
    auto transform = robot.forward_kinematics(joint_angles);
    
    Eigen::Matrix4d expected;
    expected << 0.459144, -0.846257, 0.270252, 0.677051, 0.530389, 0.505179, 0.680795, -0.299322, -0.712653, -0.169245, 0.680795, -0.615012, 0, 0, 0, 1;
    auto expected_transform = geometry::Transform(transform.get_parent(), transform.get_child(), expected);

    EXPECT_EQ(transform, expected_transform);
}

TEST(UR5_5_Suite, test_jacobian) {

    Robot::UR_5 robot = Robot::UR_5();
    auto jacobian_matrix = robot.jacobian({1., 2., 0., 0., 0.5, 2.5});
    
    Eigen::MatrixXd expected = Eigen::MatrixXd(6,6);
    expected << -0.299513, 0.322734, 0, 0.0001, 0.988692, -0.155469, 
    0.380327, 0.592271, -0.426769, 0.0001, -0.0806401, -0.554919,
    0.171612, 0.267203, -0.249694, 0.0001, -0.0806401, -0.554919, 
    -0.0210603, -0.0328571, -0.0862777, 0.0001, -0.0806401, 
    -0.554919, -0.000198682, 0.000546376, 0.000794729, 0.0001, 0.525887,
    0.721117, 4.96705e-05, -2.48353e-05, 0 , 0.0001, -0.476787, -0.246962;

    EXPECT_TRUE(jacobian_matrix.isApprox(expected, 0.01));
}

TEST(UR5_5_Suite, test_inverse_kinematics1) {

    Robot::UR_5 robot = Robot::UR_5();

    std::vector<float> joint_angles = {-1.5, 0.001, -1.5, 1.5, 2.511, -0.01};
    geometry::Transform goal = robot.forward_kinematics(joint_angles);

    auto result_joint_angles = robot.inverse_kinematics(goal);
    auto result_transform = robot.forward_kinematics(result_joint_angles);

    EXPECT_NEAR(goal.get_x(), result_transform.get_x(), 0.001);
    EXPECT_NEAR(goal.get_y(), result_transform.get_y(), 0.001);
    EXPECT_NEAR(goal.get_z(), result_transform.get_z(), 0.001);
    EXPECT_NEAR(goal.get_roll(), result_transform.get_roll(), 0.001);
    EXPECT_NEAR(goal.get_pitch(), result_transform.get_pitch(), 0.001);
    EXPECT_NEAR(goal.get_yaw(), result_transform.get_yaw(), 0.001);
}

TEST(UR5_5_Suite, test_inverse_kinematics2) {

    Robot::UR_5 robot = Robot::UR_5();

    std::vector<float> joint_angles = {1.5, 0.5, 0.5, 0.5, 2.5, -1.14};
    geometry::Transform goal = robot.forward_kinematics(joint_angles);

    auto result_joint_angles = robot.inverse_kinematics(goal);
    auto result_transform = robot.forward_kinematics(result_joint_angles);

    EXPECT_NEAR(goal.get_x(), result_transform.get_x(), 0.001);
    EXPECT_NEAR(goal.get_y(), result_transform.get_y(), 0.001);
    EXPECT_NEAR(goal.get_z(), result_transform.get_z(), 0.001);
    EXPECT_NEAR(goal.get_roll(), result_transform.get_roll(), 0.001);
    EXPECT_NEAR(goal.get_pitch(), result_transform.get_pitch(), 0.001);
    EXPECT_NEAR(goal.get_yaw(), result_transform.get_yaw(), 0.001);
}