#include <gtest/gtest.h>
#include "robots/UR5.hpp"
#include "transform/transform.hpp"

TEST(UR5_5_Suite, test_forward_kinematics1) {

    Robot::UR_5 robot = Robot::UR_5();
    auto transform = robot.forward_kinematics({0., 0., 0., 0., 0., 0.});

    Eigen::Matrix4d expected;
    expected << -1., 0., 0., 1.81725, 0., -0.707107,  0.707107, 0.11015, 0.,  0.707107, 0.707107, -0.005491, 0., 0., 0., 1.;
    auto expected_transform = geometry::Transform(transform.get_parent(), transform.get_child(), expected);

    EXPECT_EQ(transform, expected_transform);
}

TEST(UR5_5_Suite, test_forward_kinematics2) {

    Robot::UR_5 robot = Robot::UR_5();
    auto transform = robot.forward_kinematics({1., 2., 0., 0., 0.5, 2.5});
    
    Eigen::Matrix4d expected;
    expected << 0.459144, -0.846257, 0.270252, 0.677051, 0.530389, 0.505179, 0.680795, -0.299322, -0.712653, -0.169245, 0.680795, -0.615012, 0, 0, 0, 1;
    auto expected_transform = geometry::Transform(transform.get_parent(), transform.get_child(), expected);

    EXPECT_EQ(transform, expected_transform);
}