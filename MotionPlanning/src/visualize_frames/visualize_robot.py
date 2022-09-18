#!/usr/bin/env python  

import rospy
import tf2_msgs.msg
from visualization_msgs.msg import MarkerArray, Marker

from typing import List
import math
import random
import time
import threading

from transform import Transform
from link import Link
from joint import Joint
from robot import Robot
from UR5 import UR5
from mesh import Mesh


class VisualizeRobot:
    """  """

    def __init__(self, robot: Robot):
        """ Constructor. Any future changes to robot object will be shown in RVIZ

            Args:
                robot (Robot): The robot to visualize
        """

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_meshes = rospy.Publisher('/robot_meshes', MarkerArray, queue_size=1)
        self.rate = rospy.Rate(20)
        
        self.display_robot_updates_thread = threading.Thread(target=self.display_robot_updates)
        self.display_robot_updates_thread.start()


    def display_robot_updates(self):
        """ Display changes to the robot's state in RVIZ
        
            Target function for thread which will continously
            update the robot model in RVIZ when there are 
            changes to self.robot
        """

        while not rospy.is_shutdown():
            
            self.update_frames(robot)
            self.update_mesh_visualization(robot)            
            self.rate.sleep()
    
    def update_mesh_visualization(self, robot: Robot):
        """ Remove prior meshes and add the new meshes for new joint angles

            Args:
                robot (Robot) - the robot whose meshes we are visualizing
         """

        self.delete_prior_mesh_visualization()
        robot_visualization = Mesh.create_visualization(robot.links)
        self.pub_meshes.publish(robot_visualization)

    def update_frames(self, robot: Robot):
        """ Update the TF tree based on the robot's current joint angles

            Args:
                robot (Robot) - the robot whose frames we are visualizing
         """

        transforms = robot.get_transforms() 
        ros_transforms = [transform.to_ros() for transform in transforms]
        tfm = tf2_msgs.msg.TFMessage(ros_transforms)
        self.pub_tf.publish(tfm)

    def delete_prior_mesh_visualization(self):
        """ Remove all meshes in RVIZ """

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.action = marker.DELETEALL 
        markerArray.markers.append(marker)
        self.pub_meshes.publish(markerArray)


if __name__ == '__main__':
    rospy.init_node('visualize_UR5_with_TF2')
    time.sleep(1) # Let the node get setup

    robot = UR5()
    robot_Visualizer = VisualizeRobot(UR5)
    
    i = 0
    while True:

        robot.set_joints([i, 0, 0, 0, 0, 0])
        i += 0.01
        time.sleep(0.1)


    rospy.spin()