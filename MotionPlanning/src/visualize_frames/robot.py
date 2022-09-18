"""Describe me """

from typing import List

from transform import Transform
from link import Link
from joint import Joint

class Robot:
    """ Base class for specefic robots to inherit from """

    def __init__(self):
        self.links = self.get_all_links()
        self.joints = self.get_all_joints()
        self.joint_angles = [0.0] * len(self.joints)

    def set_joints(self, joint_angles: List[Joint]):
        """ """
        
        for joint, joint_angle in zip(self.joints, joint_angles):
            joint.apply_rotation(joint_angle)

    def get_transforms(self) -> List[Transform]:
        """" """

        link_transforms = [link.transform for link in self.links]
        joint_transforms = [joint.transform for joint in self.joints]
        return link_transforms + joint_transforms
    