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


    Robot::Robot1* robot = new Robot::UR_5();
    auto start_state = planning::JointState({3.14, -0.5, -1., 1.57/4, 1.57, 1.0});
    auto goal_state = planning::JointState({0., 1.2, 1., 0., -1.57, -1.57});
    planning::PlanningRequest planning_request = planning::PlanningRequest(&start_state, &goal_state);

    ros::init(argc, argv, "visualizer2");

    auto rrt = planning::RRT(robot, planning_request);

    auto path = rrt.solve();

    std::vector<float> schedule = std::vector<float>();
    for (int i = 0; i < path->size(); i++){
        schedule.push_back(i/2.0);
    }

    auto trajectory = planning::Trajectory(*path, schedule);
    auto ros_trajectory = ros_conversions::to_ros(trajectory);


    geometry::ROSRobotTrajectoryVisualizer trajectory_visualizer = geometry::ROSRobotTrajectoryVisualizer(robot, trajectory, "/trajectory", 0.1);


    trajectory_visualizer.visualize();










    //std::cout << "The size of the graph is " << graph.size() << std::endl;


    /*
    using namespace geometry;

    // construct transform tree
    Frame root = Frame("world");
    StaticTransformTree tree = StaticTransformTree(root);

    auto transforms = robot->get_ordered_transforms();
    for (int i = 0; i < transforms.size(); i++) {
        tree.add(transforms[i]);
    }
    
    
    ROSRobotVisualizer visualizer = ROSRobotVisualizer(robot, "/tf", 1.);
    visualizer.visualize();

    while (true) {

    for (int i = 0; i < graph.size(); i++) {

        visualizer.set_joint_angles(graph[i]->get_configuration());

         // need to update the TransformTree
         auto transforms = robot->get_ordered_transforms();
         for (int i = 0; i < transforms.size(); i++) {
            
            auto transform = transforms[i];
            tree.set_transform(transform, transform.getPosition(), transform.getEulerAngles());
         }
         

        sleep(0.8);
    }
    }
    */

    return 0;
}