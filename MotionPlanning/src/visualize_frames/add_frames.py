#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


class VisualizeUR5:
    """ Describe me """

    def __init__(self):
        """ Describe me """
    
        i = 0

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            frames = self.get_all_frames(i)
            tfm = tf2_msgs.msg.TFMessage(frames)
            self.pub_tf.publish(tfm)
            i += 0.1

    
    def get_base_transform(self, yaw):
        """ """

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "link1"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "link2"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 2.0
        t.transform.translation.z = 2.0

        quaterion_as_array = self.euler_to_quaternion(0, 0, yaw)
        t.transform.rotation.x = quaterion_as_array[0]
        t.transform.rotation.y = quaterion_as_array[1]
        t.transform.rotation.z = quaterion_as_array[2]
        t.transform.rotation.w = quaterion_as_array[3]

        return t


    def get_(self):
        """ """
        ...


    def get_all_frames(self, yaw: int):
        """ """
        return [self.get_base_transform(yaw)]
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """ """
        
        # RPY to convert: 90deg, 0, -90deg
        return quaternion_from_euler(roll, pitch, yaw)

        


if __name__ == '__main__':
    rospy.init_node('visualize_UR5_with_TF2')
    UR5_Visualizer = VisualizeUR5()
    rospy.spin()
