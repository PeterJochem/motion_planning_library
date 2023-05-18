#include "planning/rrt.hpp"
#include "planning/rrt_star.hpp"
#include "planning/rrt_connect.hpp"
#include "robots/UR5.hpp"
#include "planning/joint_state.hpp"
#include "visualizer/ROS_robot_visualizer.hpp"
#include "visualizer/ROS_robot_trajectory_visualizer.hpp"
#include "planning/trajectory.hpp"
#include "planning/naive_time_scaler.hpp"
#include "ros_conversions/ros_conversions.hpp"
#include "trajectory_msgs/JointTrajectory.h"
#include <unistd.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "visualizer2");

    // Specify the planning problem.
    Robot::Robot1* robot = new Robot::UR_5();
    auto start_state = planning::JointState({1.0, 0., 0.0, 0.2, 0.1, 0.});
    auto goal_state = planning::JointState({0., 0., 0., 0., 0, 0.});
    planning::RRTPlanningRequest planning_request = planning::RRTPlanningRequest(&start_state, &goal_state, 100000, 0.1);

    // Solve the planning problem.
    //auto rrt = planning::RRT(robot, &planning_request);
    //auto rrt = planning::RRTStar(robot, &planning_request);
    auto rrt = planning::RRTConnect(robot, &planning_request);
    auto path = rrt.solve();
    auto time_scaler = planning::NaiveTimeScaler(*path);
    auto trajectory = time_scaler.scale();

    // Visualize the trajectory in RVIZ.
    geometry::ROSRobotTrajectoryVisualizer trajectory_visualizer = geometry::ROSRobotTrajectoryVisualizer(robot, trajectory, "/trajectory", 0.1);
    trajectory_visualizer.visualize();

    return 0;
}