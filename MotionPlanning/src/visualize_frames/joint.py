from typing import List
from transform import Transform

class Axis:
    """ Represents the direction of rotation about a joint. 

        For more details, see http://wiki.ros.org/urdf/XML/joint
     """

    def __init__(self, vector: List[float]):
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


class Joint:
    """ Represents a robot's joint and the movements it can undergo """

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float,\
     yaw: float, axis: Axis, lower_limit: float, upper_limit: float):
        """ Constrcutor 
        
            Args:
                parent_frame (str): Name of parent frame with which we measure the child frame
                child_frame (str): Name of the frame which defines the link
                x (float): X position of link in parent frame in meters
                y (float): Y position of link in parent frame in meters
                z (float): Z position of link in parent frame in meters
                roll (float): Euler angle around parent's X-axis
                pitch (float): Euler angle around parent's Y-axis
                yaw (float): Euler angle around parent's Z-axis
                axis (Axis): Axis of rotation for the joint
                lower_limit (float): Lower joint limit
                upper_limit (float): Upper joint limit
        """

        self.parent_frame = parent_frame
        self.child_frame = child_frame
        
        self.x = x
        self.y = y
        self.z = z
        
        self.zero_angle_roll = roll
        self.zero_angle_pitch = pitch
        self.zero_angle_yaw = yaw

        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        self.zero_angle_transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.axis = axis


    def apply_rotation(self, radians: float):
        """ Rotate the joint frame around its axis of rotation by the number of radians 

            Args:
                radians (float): The number of radians from the zero angle to rotate the joint
        """

        if self.axis.is_pure_roll():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, \
             self.zero_angle_roll + radians, self.zero_angle_pitch, self.zero_angle_yaw)
        
        elif self.axis.is_pure_pitch():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, \
            self.zero_angle_roll, self.zero_angle_pitch + radians, self.zero_angle_yaw)

        elif self.axis.is_pure_yaw():
            self.transform = Transform(self.parent_frame, self.child_frame, self.x, self.y, self.z, \
            self.zero_angle_roll, self.zero_angle_pitch, self.zero_angle_yaw + radians)

        else:
            raise Exception("Cannot apply rotation since the axis of rotation is not a pure roll, pitch, or yaw.")