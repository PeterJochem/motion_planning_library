#include "planning/rrt.hpp"
#include "robots/UR5.hpp"
#include "planning/joint_state.hpp"
#include "visualizer/ROS_robot_visualizer.hpp"
#include "visualizer/ROS_robot_trajectory_visualizer.hpp"
#include "planning/trajectory.hpp"
#include "ros_conversions/ros_conversions.hpp"
#include "trajectory_msgs/JointTrajectory.h"
#include <unistd.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "visualizer2");

    Robot::Robot1* robot = new Robot::UR_5();
    auto start_state = planning::JointState({1.0, 0., 0.0, 0., 0., 0.});
    auto goal_state = planning::JointState({0., 0.2, 0., 0., 0, 0.});


    planning::RRTPlanningRequest planning_request = planning::RRTPlanningRequest(&start_state, &goal_state, 100, 0.1);


    auto rrt = planning::RRT(robot, &planning_request);
    auto path = rrt.solve();

    // Temporary way to convert a path to a trajectory.
    // auto time_scaler = NaiveTimeScaler();
    // auto trajectory = time_scaler.scale(path); 
    std::vector<float> schedule = std::vector<float>();
    for (int i = 0; i < path->size(); i++){
        schedule.push_back(i/2.0);
    }

    auto trajectory = planning::Trajectory(*path, schedule);

    geometry::ROSRobotTrajectoryVisualizer trajectory_visualizer = geometry::ROSRobotTrajectoryVisualizer(robot, trajectory, "/trajectory", 0.1);
    trajectory_visualizer.visualize();

    return 0;
}