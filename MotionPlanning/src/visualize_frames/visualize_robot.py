#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import MarkerArray, Marker


class Link:
    """ Describe me """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_file=None):
        """ Describe me """

        self.transform = geometry_msgs.msg.TransformStamped()
        self.transform.header.frame_id = parent_frame
        self.transform.header.stamp = rospy.Time.now()
        self.transform.child_frame_id = child_frame
        self.transform.transform.translation.x = x
        self.transform.transform.translation.y = y
        self.transform.transform.translation.z = z

        quaterion_as_array = self.euler_to_quaternion(roll, pitch, yaw)
        
        self.transform.transform.rotation.x = quaterion_as_array[0]
        self.transform.transform.rotation.y = quaterion_as_array[1]
        self.transform.transform.rotation.z = quaterion_as_array[2]
        self.transform.transform.rotation.w = quaterion_as_array[3]
        
        self.mesh_file_path = mesh_file

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """ """

        # RPY to convert: 90deg, 0, -90deg
        return quaternion_from_euler(roll, pitch, yaw)



class UR5:
    """ """
    ...
    # List of frame objects 


class VisualizeUR5:
    """ Describe me """

    def __init__(self):
        """ Describe me """

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.pub_meshes = rospy.Publisher('/robot_meshes', MarkerArray, queue_size=1)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            
            link = self.get_all_links()
            tfm = tf2_msgs.msg.TFMessage([link.transform for link in link])
            self.pub_tf.publish(tfm)

            # meshes = 
            #self.pub_meshes.publish(frames)

    def get_meshes(frames):
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


    def define_base_link(self):
        """ """
        
        return Link("world", "base_link", x=0.0, y=2.0, z=2.0, roll=0, pitch=0, yaw=0, mesh_file=None)


    def define_shoulder_link(self):
        """ """
        
        return Link("base_shoulder_joint", "shoulder_link", x=0.0, y=2.0, z=2.0, roll=0, pitch=0, yaw=0, mesh_file=None)

    
            


    def get_all_links(self):
        """ """
        
        # define base link
        # define shoulder joint
        # define 

        # You actually want to do this recursively

        base_link = self.define_base_link()
        
        # base_shoulder_joint = self.get_base_shoulder_joint_transform()

        shoulder_link = self.define_shoulder_link()
    
        
        return [base_link, shoulder_link]
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """ """
        
        # RPY to convert: 90deg, 0, -90deg
        return quaternion_from_euler(roll, pitch, yaw)



if __name__ == '__main__':
    rospy.init_node('visualize_UR5_with_TF2')
    UR5_Visualizer = VisualizeUR5()
    rospy.spin()
