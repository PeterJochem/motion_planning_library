from transform import Transform
from mesh import Mesh

class Link:

    def __init__(self, parent_frame: str, child_frame: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float, mesh_file=None):
        """ Constructor

            Args:
                parent_frame (str): Name of parent frame with which we measure the child frame
                child_frame (str): Name of the frame which defines the link
                x (float): X position of link in parent frame in meters
                y (float): Y position of link in parent frame in meters
                z (float): Z position of link in parent frame in meters
                roll (float): Euler angle around parent's X-axis
                pitch (float): Euler angle around parent's Y-axis
                yaw (float): Euler angle around parent's Z-axis
        """

        self.transform = Transform(parent_frame, child_frame, x, y, z, roll, pitch, yaw)
        self.mesh_file_path = mesh_file
        self.mesh = Mesh(mesh_file, child_frame, self.transform) 

    def get_mesh(self):
        """ """

        return self.mesh