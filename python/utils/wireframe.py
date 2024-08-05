import numpy as np
import math
from .orientation import eul2dcm, right_hand_rule
from .transformation import NED2ENU

# vertice indexs:
#           5-------7
#           | \     |\
#           |  1------3
#           |  |    | |
#           4--|----6 |
#            \ |     \|
#              0------2

class Vertice:
    """
    Vertice of Cube

    :param tuple coordinates: xyz axis vertice coordinates
    :param tuple color: color
    """
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color

    def __str__(self):
        return "Vertice x: {}\nVertice y: {}\nVertice z: {}\nVertice color: {}".format(self.x, self.y, self.z, self.color)

class Face:
    """
    Face of Cube

    :param tuple vertices: vertice index combined into face
    :param tuple color: color
    """
    def __init__(self, vertices, color):
        self.vertice_indexs = vertices
        self.color = color

    def __str__(self):
        return "Face vertice_indexs: {}\nFace color: {}".format(self.vertice_indexs, self.color)

class wireframe:
    """
    Generate wireframe as IMU model included 8 vertices, 12 edges, and 6 faces [1]_ [2]_

    :param str nav_frame: navigation frame

    axis color: \n
    Positive of x axis is Red, Negative of x axis is Cyan \n
    Positive of y axis is Green, Negative of y axis is Magenta \n
    Positive of z axis is Blue, Negative of z axis is Yellow \n

    .. Reference
    .. [1] 'Cube property references <https://en.wikipedia.org/wiki/Cube>'
    .. [2] 'Github references <https://github.com/DonovanZhu/9DoF_MARG_Madgwick_Filter/tree/master>'
    """
    def __init__(self, nav_frame):
        self.vertices = []
        self.edges = []
        self.faces = []
        self.nav_frame = nav_frame

        # Check parameter
        if (self.nav_frame != "ENU") and (self.nav_frame != "NED"):
            raise ValueError("Navigation frame should be either ENU or NED")
        if self.nav_frame == "ENU":
            self.rotation_seq = "zxy"
        elif self.nav_frame == "NED":
            self.rotation_seq = "zyx"

    def initialize_cube(self, max_x, min_x, max_y, min_y, max_z, min_z):
        """
        Initialize the IMU model of the specified size

        :param int max_x: maximum x axis cube coordinates
        :param int min_x: minimum x axis cube coordinates
        :param int max_y: maximum y axis cube coordinates
        :param int min_y: minimum y axis cube coordinates
        :param int max_z: maximum z axis cube coordinates
        :param int min_z: minimum z axis cube coordinates
        """
        cube_vertices = [(x, y, z) for x in (min_x, max_x) for y in (min_y, max_y) for z in (min_z, max_z)]
        vertice_colors = [(255, 255, 255)] * len(cube_vertices)
        self.add_vertices(cube_vertices, vertice_colors)
        if self.nav_frame=="ENU":
            cube_faces = [(1, 5, 7, 3), (0, 4, 6, 2), (4, 5, 7, 6), (0, 1, 3, 2), (6, 7, 3, 2), (4, 5, 1, 0)]
            face_colors = [(0, 0, 255), (255, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 255, 0)]
        elif self.nav_frame=="NED":
            cube_faces = [(1, 5, 7, 3), (0, 4, 6, 2), (4, 5, 7, 6), (0, 1, 3, 2), (6, 7, 3, 2), (4, 5, 1, 0)]
            face_colors = [(0, 0, 255), (255, 255, 0), (0, 255, 0), (255, 0, 255), (0, 255, 255), (255, 0, 0)]
        self.add_faces(cube_faces, face_colors)
        
    def add_vertices(self, vertice_list, color_list):
        """
        Add vertice into IMU model

        :param list vertice_list: IMU model vertice coordinates
        :param list color_list: list stored each vertice color
        """
        for vertice, color in zip(vertice_list, color_list):
            self.vertices.append(Vertice(vertice, color))

    def add_faces(self, face_list, color_list):
        """
        Add face into IMU model

        :param list face_list: IMU model face index
        :param list color_list: list stored each face color
        """
        for face, color in zip(face_list, color_list):
            self.faces.append(Face(face, color))

    def update_attitude(self, rpy):
        """
        Update IMU model attitude

        :param ndarray rpy: roll, pitch, yaw angle in degree
        """
        self.roll = math.radians(rpy[0][0])
        self.pitch = math.radians(rpy[1][0])
        self.yaw = math.radians(rpy[2][0])

    def rotate_point(self, point):
        """
        Rotate the IMU vertice coordinates

        :param ndarray point: xyz position
        :returns: 
            - new_point (numpy.matrix) - rotated coordinates
        """
        roll, pitch, yaw = self.roll, self.pitch, self.yaw
        if self.nav_frame == "ENU":
            DCM = eul2dcm(-roll, pitch, -yaw, seq=self.rotation_seq, coordinates="right")
        elif self.nav_frame == "NED":
            # When pitch angle pass through 90 degrees threshold
            # the display of the IMU model pitch angle rotation
            # reversed sign
            roll, pitch, yaw = NED2ENU(roll, pitch, yaw)
            DCM = eul2dcm(-roll, pitch, -yaw, seq=self.rotation_seq, coordinates="right")
        new_point = DCM @ point
        return new_point
