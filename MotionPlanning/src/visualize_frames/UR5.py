
import math
from robot import Robot
from link import Link
from joint import Joint, Axis
from transform import Transform 
from utilities import euler_to_quaternion
from mesh import Mesh

class UR5(Robot):
    """ """
    
    def __init__(self):
        self.mesh_file_path = "/home/pj/MotionPlanning/MotionPlanning/src/mesh_files/dae"
        super().__init__()
        
    def define_frame(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_file=None):
        """ Describe me """

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = parent_frame
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = euler_to_quaternion(roll, pitch, yaw)
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
        return Joint("base_link", "base_shoulder_joint", 0, 0, 0.089159, 0, 0, 0, axis, -2 * math.pi, 2 * math.pi)

    def define_shoulder_upper_arm_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("shoulder_link", "shoulder_upper_arm_joint", 0, 0.13585, 0, 0, math.pi/2, 0, axis, -2 * math.pi, 2 * math.pi)

    def define_upper_arm_forearm_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("upper_arm_link", "upper_arm_forearm_joint", 0, -0.1197, 0.42500, 0, 0, 0, axis, -2 * math.pi, 2 * math.pi)

    def define_forearm_wrist1_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("forearm_link", "forearm_wrist1_joint", 0, 0, 0.39225, 0, math.pi/2, 0, axis, -2 * math.pi, 2 * math.pi)

    def define_wrist1_wrist2_joint(self):
        """ """

        axis = Axis([0,0,1])
        return Joint("wrist1_link", "wrist1_wrist2_joint", 0, 0.093, 0.0, 0, 0, 0, axis, -2 * math.pi, 2 * math.pi)

    def define_wrist2_wrist3_joint(self):
        """ """

        axis = Axis([0,1,0])
        return Joint("wrist2_link", "wrist2_wrist3_joint", 0, 0.0, 0.09465, 0, 0, 0, axis, -2 * math.pi, 2 * math.pi)

    
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