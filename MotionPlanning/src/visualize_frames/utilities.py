""" For random utilities without a clear home """
 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """ Convert from Euler angles to a quaternion

        Args:
            roll (float) - rotation about X-axis in radians
            pitch (float) - rotation about Y-axis in radians 
            yaw (float) - rotation about Z-axis in radians

        Returns:
            geometry_msgs Quaternion
    """
        
    quaterion_as_array = quaternion_from_euler(roll, pitch, yaw)
    q = Quaternion()
    q.x = quaterion_as_array[0]
    q.y = quaterion_as_array[1]
    q.z = quaterion_as_array[2]
    q.w = quaterion_as_array[3]

    return q


