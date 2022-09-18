import rospy
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import PoseStamped
import geometry_msgs.msg
from typing import List
from utilities import euler_to_quaternion


class Transform:
    """ Represents a position and orienation in space measured in its parent frame"""

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        """ Constrcutor 
        
            Args:
                parent_frame (str) - Name of frame this transform is measured in
                child_frame (str) - Name of the resulting frame
                x: (float) - X position of transform in meters measured in parent frame 
                y (float) - Y position of transform in meters measured in parent frame
                z (float) -  Z position of transform in meters in parent frame
                roll (float) -  Roll in radians measured in the parent frame 
                pitch (float) - Pitch in radians measured in the parent frame
                yaw (float) - Yaw in radians measured in the parent frame
        """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


    def to_ros(self) -> geometry_msgs.msg.TransformStamped:
        """ Convert this Transform to a format displayable in RVIZ 
        
            Returns:
                This transform as a geometry_msgs.msg.TransformStamped
        """

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.header.stamp = rospy.Time.now()
        transform.child_frame_id = self.child_frame
        
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = self.z

        transform.transform.rotation = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        return transform


    def transform_pose(self, input_pose, from_frame: str, to_frame: str):
        """ I don't think I need this. You just need to make a pose stamped and say the base pose object
        is defined in the from_frame. Then, RVIZ will take care of rendering it in the base frame - the world"""

        pose_stamped = PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()
        return pose_stamped.pose