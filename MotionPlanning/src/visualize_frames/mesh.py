from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose
from os.path import exists
import random
from typing import List
from transform import Transform

class Mesh:
    """ """

    def __init__(self, mesh_file: str, parent_frame: str, transform: Transform):

        if not exists(mesh_file):
            raise Exception(f"Can not find mesh file for mesh defined in {parent_frame} frame.")

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

        return int(random.getrandbits(31))

    def to_ros(self):   
        """ """

        return self.marker


    @staticmethod
    def create_visualization(links: List):
        """ """

        array = MarkerArray()
        for link in links:
            mesh = link.get_mesh()
            if mesh is not None and mesh.has_valid_pose():
                array.markers.append(mesh.to_ros())
        return array