#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from rosgraph_msgs.msg import Log


import math
import uuid
import random
import time
import threading

class Axis:
    """ """

    def __init__(self, vector):
        self.vector = vector

    def is_pure_roll(self) -> bool:
        """ """

        return self.vector[0] == 1

    def is_pure_pitch(self) -> bool:
        """ """

        return self.vector[1] == 1

    def is_pure_yaw(self) -> bool:
        """ """

        return self.vector[2] == 1




class Transform:
    """ """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """ """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """ """

        # RPY to convert: 90deg, 0, -90deg
        return quaternion_from_euler(roll, pitch, yaw)

    def to_ros(self):
        """ """

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        quaterion_as_array = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        
        transform.transform.rotation.x = quaterion_as_array[0]
        transform.transform.rotation.y = quaterion_as_array[1]
        transform.transform.rotation.z = quaterion_as_array[2]
        transform.transform.rotation.w = quaterion_as_array[3]

        return transform


    def transform_pose(self, input_pose, from_frame: str, to_frame: str):
        """ I don't think I need this. You just need to make a pose stamped and say the base pose object
        is defined in the from_frame. Then, RVIZ will take care of rendering it in the base frame - the world"""

        pose_stamped = PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()
        return pose_stamped.pose

        #########

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            
            #listener.waitForTransform("/frame1", "/frame2", rospy.Time(), rospy.Duration(4.0))
            
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, from_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except Exception as e:
            print(e)
            print(f"Failed to find transform from {from_frame} to {to_frame}. It's normal for this to happen occasionally.")
            return None

class Mesh:
    """ """

    def __init__(self, mesh_file: str, parent_frame: str, transform):

        self.mesh_file = mesh_file
        self.parent_frame = parent_frame
        self.transform = transform

        self.marker = Marker()
        self.marker.id = self.random_id()
        self.marker.mesh_resource = f"file:///{mesh_file}"
        self.marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        self.marker.type = self.marker.MESH_RESOURCE
        self.marker.header.frame_id = parent_frame # Set from the input argument
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        
        self.mesh_pose = Pose() # The identity pose?
        self.mesh_pose.orientation.x = 0.0
        self.mesh_pose.orientation.y = 0.0
        self.mesh_pose.orientation.z = 0.0
        self.mesh_pose.orientation.w = 1.0

        #self.marker.pose = transform.transform_pose(self.mesh_pose, parent_frame, parent_frame)# "world")
        self.marker.pose = transform.transform_pose(self.mesh_pose, parent_frame, "world")


    def set_pose(self):
        """ """
        ...

    def has_valid_pose(self) -> bool:
        """ """

        return self.marker.pose is not None


    def random_id(self) -> int:
        """ """

        return int(random.random() * 1000) # int(uuid.uuid1())

    def to_ros(self):   
        """ """

        return self.marker

    @staticmethod
    def create_visualization(links):
        """ """

        array = MarkerArray()
        for link in links:
            mesh = link.get_mesh()
            if mesh is not None and mesh.has_valid_pose():
                array.markers.append(mesh.to_ros())
        return array


class Link:
    """ Describe me """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_file=None):
        """ Describe me """

        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.mesh_file_path = mesh_file
        self.mesh = Mesh(mesh_file, child_frame, self.transform) 

    def get_mesh(self):
        """ """

        return self.mesh



    
class Joint:
    """ """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, axis):
        """ """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


        self.zero_angle_transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.axis = axis


    def apply_rotation(self, radians: float):
        """Rotate the joint frame around its axis of rotation by the number of radians """

        if self.axis.is_pure_roll():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, self.roll + radians, self.pitch, self.yaw)
        
        elif self.axis.is_pure_pitch():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, self.roll, self.pitch + radians, self.yaw)

        elif self.axis.is_pure_yaw():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, self.roll, self.pitch, self.yaw + radians)

        else:
            raise Exception("Cannot apply rotation since the axis of rotation is not a pure roll, pitch, or yaw.")



class UR5:
    """ """
    ...
    # List of frame objects 


class VisualizeUR5:
    """ Describe me """

    def __init__(self):
        """ Describe me """


        rospy.Subscriber("/rosout_agg", Log, self.callback)

        self.mesh_file_path = "/home/pj/MotionPlanning/MotionPlanning/src/mesh_files/dae"
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_meshes = rospy.Publisher('/robot_meshes', MarkerArray, queue_size=1)
        rate = rospy.Rate(20)

        ############## This defines the robot
        links = self.get_all_links()
        joints = self.get_all_joints()
        ##############

        loop_count = 0
        while not rospy.is_shutdown():
            
            rate.sleep()
            
            joint_angles = [loop_count/100, loop_count/1000, 0.5, loop_count * 0.02, 0, 0]

            self.update_joints(joints, joint_angles)
            self.update_frames(links, joints)
            self.update_mesh_visualization(links)
            
            loop_count += 1

    def update_mesh_visualization(self, links):
        """ """

        self.delete_prior_mesh_visualization()
        robot_visualization = Mesh.create_visualization(links)
        self.pub_meshes.publish(robot_visualization)



    def update_frames(self, links, joints):
        """ """

        transforms = [link.transform.to_ros() for link in links] + [joint.transform.to_ros() for joint in joints]
        tfm = tf2_msgs.msg.TFMessage(transforms)
        self.pub_tf.publish(tfm)


    def create_mesh_visualization(self, links):
        """ """
        
        array = MarkerArray()
        for link in links:
            mesh = link.get_mesh()
            if mesh is not None and mesh.has_valid_pose():
                array.markers.append(mesh.to_ros())

        return array



    def delete_prior_mesh_visualization(self):
        """ """

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.action = marker.DELETEALL # send the DELETEALL marker to delete all marker in RViz
        markerArray.markers.append(marker)
        self.pub_meshes.publish(markerArray)


    def update_joints(self, joints, joint_angles):
        """ """

        if joints is None or joint_angles is None or len(joint_angles) != len(joints):
            raise Exception("Illegal input to update joints.")

        for joint, joint_angle in zip(joints, joint_angles):
            joint.apply_rotation(joint_angle)


    def callback(self, data):

        print("\n\nSubscriber ran\n\n")


    def get_meshes(self, frames):
        ...

        # Traverse the list of frames 
        # If a mesh is defined in this frame, then update the position info
        # 

        # return list of meshes with their position info


    def define_frame(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_file=None):
        """ Describe me """

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        quaterion_as_array = self.euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = quaterion_as_array[0]
        t.transform.rotation.y = quaterion_as_array[1]
        t.transform.rotation.z = quaterion_as_array[2]
        t.transform.rotation.w = quaterion_as_array[3]

        return t

  
    def define_base_link(self) -> Link:
        """ """
        
        mesh_file = self.mesh_file_path + "/base.dae"
        return Link("world", "base_link", 1.0, 0.0, 0, 0, 0, 0, mesh_file)


    def define_shoulder_link(self) -> Link:
        """ """
        
        mesh_file = self.mesh_file_path + "/shoulder.dae"
        return Link("base_shoulder_joint", "shoulder_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)

    
    def define_upper_arm_link(self) -> Link: 
        """ """

        mesh_file = self.mesh_file_path + "/upperarm.dae"
        return Link("shoulder_upper_arm_joint", "upper_arm_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)

    def define_forearm_link(self):
        """ """

        mesh_file = self.mesh_file_path + "/forearm.dae"
        return Link("upper_arm_forearm_joint", "forearm_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)

    def define_wrist1_link(self):
        """ """

        mesh_file = self.mesh_file_path + "/wrist1.dae"
        return Link("forearm_wrist1_joint", "wrist1_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)

    def define_wrist2_link(self):
        """ """

        mesh_file = self.mesh_file_path + "/wrist2.dae"
        return Link("wrist1_wrist2_joint", "wrist2_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)

    def define_wrist3_link(self):
        """ """

        mesh_file = self.mesh_file_path + "/wrist3.dae"
        return Link("wrist2_wrist3_joint", "wrist3_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mesh_file)


    def define_base_shoulder_joint(self):
        """ """

        axis = Axis([0,0,1])
        return Joint("base_link", "base_shoulder_joint", 0, 0, 0.089159, 0, 0, 0, axis)

    def define_shoulder_upper_arm_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("shoulder_link", "shoulder_upper_arm_joint", 0, 0.13585, 0, 0, math.pi/2, 0, axis)

    def define_upper_arm_forearm_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("upper_arm_link", "upper_arm_forearm_joint", 0, -0.1197, 0.42500, 0, 0, 0, axis)

    def define_forearm_wrist1_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("forearm_link", "forearm_wrist1_joint", 0, 0, 0.39225, 0, math.pi/2, 0, axis)

    def define_wrist1_wrist2_joint(self):
        """ """

        axis = Axis([0,0,1])
        return Joint("wrist1_link", "wrist1_wrist2_joint", 0, 0.093, 0.0, 0, 0, 0, axis)

    def define_wrist2_wrist3_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("wrist2_link", "wrist2_wrist3_joint", 0, 0.0, 0.09465, 0, 0, 0, axis)

    


    def get_all_joints(self):
        """ """

        base_shoulder_joint = self.define_base_shoulder_joint()
        shoulder_upper_arm_joint = self.define_shoulder_upper_arm_joint()
        upper_arm_forearm_link = self.define_upper_arm_forearm_joint()
        forearm_wrist1_joint = self.define_forearm_wrist1_joint()
        wrist1_wrist2_joint = self.define_wrist1_wrist2_joint()
        wrist2_wrist3_joint = self.define_wrist2_wrist3_joint()


        return [base_shoulder_joint, shoulder_upper_arm_joint, upper_arm_forearm_link, \
             forearm_wrist1_joint, wrist1_wrist2_joint, wrist2_wrist3_joint]
        


    def get_all_links(self):
        """ """

        base_link = self.define_base_link()
        shoulder_link = self.define_shoulder_link()
        upper_arm_link = self.define_upper_arm_link()
        forearm_link = self.define_forearm_link()
        wrist1_link = self.define_wrist1_link()
        wrist2_link = self.define_wrist2_link()
        wrist3_link = self.define_wrist3_link()

        return [base_link, shoulder_link, upper_arm_link, forearm_link, wrist1_link, wrist2_link, wrist3_link]
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """ """
        
        # RPY to convert: 90deg, 0, -90deg
        return quaternion_from_euler(roll, pitch, yaw)



if __name__ == '__main__':
    rospy.init_node('visualize_UR5_with_TF2')
    time.sleep(2)
    x = threading.Thread(target=VisualizeUR5 )
    x.start()
    #UR5_Visualizer = VisualizeUR5()
    rospy.spin()
