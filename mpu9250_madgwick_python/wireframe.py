import numpy as np
import math
from orientation import eul2dcm, right_hand_rule

class Vertice:
    def __init__(self, coordinates, color):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]
        self.color = color

    def __str__(self):
        return "Vertice x: {}\nVertice y: {}\nVertice z: {}\nVertice color: {}".format(self.x, self.y, self.z, self.color)

# Face stores 4 vertices that make up a face of the block
class Face:
    def __init__(self, vertices, color):
        self.vertice_indexs = vertices
        self.color = color

    def __str__(self):
        return "Face vertice_indexs: {}\nFace color: {}".format(self.vertice_indexs, self.color)

class wireframe:
    def __init__(self, nav_frame):
        # Cube property references (https://en.wikipedia.org/wiki/Cube)
        # Github references (https://github.com/DonovanZhu/9DoF_MARG_Madgwick_Filter/tree/master)
        # 8 vertices
        # 12 edges
        # 6 faces

        # x-axis: Red(+), Cyan(-)
        # y-axis: Green(+), Magenta(-)
        # z-axis: Blue(+), Yellow(-)
        #     ENU               NED
        # y    z                   x
        #  \   |                  /
        #   \  |                 ------- y 
        #    \ |                 |
        #     \|------- x        z

        #           5-------7
        #           | \     |\
        #           |  1------3  
        #           |  |    | |
        #           4--|----6 |
        #            \ |     \|
        #              0------2

        self.vertices = []
        self.edges = []
        self.faces = []
        self.rpy = []
        self.nav_frame = nav_frame
        if (self.nav_frame != "ENU") and (self.nav_frame != "NED"):
            raise ValueError("Navigation frame should be either ENU or NED")

    def initialize_cube(self, max_x, min_x, max_y, min_y, max_z, min_z):
        cube_vertices = [(x, y, z) for x in (min_x, max_x) for y in (min_y, max_y) for z in (min_z, max_z)]
        vertice_colors = [(255, 255, 255)] * len(cube_vertices)
        self.add_vertices(cube_vertices, vertice_colors)
        if self.nav_frame=="ENU":
            cube_faces = [(1, 5, 7, 3), (0, 4, 6, 2), (4, 5, 7, 6), (0, 1, 3, 2), (6, 7, 3, 2), (4, 5, 1, 0)]
            # face_colors = [(0, 0, 255), (255, 255, 0), (0, 255, 0), (255, 0, 255), (0, 255, 255), (255, 0, 0)]
            face_colors = [(0, 0, 255), (255, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (0, 255, 0)]
        elif self.nav_frame=="NED":
            cube_faces = [(0, 4, 6, 2), (1, 5, 7, 3), (6, 7, 3, 2), (4, 5, 1, 0), (4, 5, 7, 6), (0, 1, 3, 2)]
            face_colors = [(0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 0), (0, 255, 0), (255, 0, 255)]
        self.add_faces(cube_faces, face_colors)

    def add_vertices(self, vertice_list, color_list):
        for vertice, color in zip(vertice_list, color_list):
            self.vertices.append(Vertice(vertice, color))

    def add_faces(self, face_list, color_list):
        for face, color in zip(face_list, color_list):
            self.faces.append(Face(face, color))

    def update_attitude(self, rpy):
        roll = math.radians(rpy[0][0])
        pitch = math.radians(rpy[1][0])
        yaw = math.radians(rpy[2][0])
        rpy = np.array([[roll],[pitch],[yaw]])
        self.rpy = rpy

    def rotate_point(self, point):
        if self.nav_frame == "ENU":
            maxtrix = right_hand_rule.euler_x_rotation(math.radians(-180))
            new_rpy = maxtrix @ self.rpy
            roll = -new_rpy.tolist()[0][0]
            pitch = -new_rpy.tolist()[1][0]
            yaw = new_rpy.tolist()[2][0]
            # maxtrix = right_hand_rule.euler_x_rotation(math.radians(0))
            # new_rpy = maxtrix @ self.rpy
            # roll = new_rpy.tolist()[0][0]
            # pitch = new_rpy.tolist()[1][0]
            # yaw = new_rpy.tolist()[2][0]
            DCM = eul2dcm(roll, pitch, yaw, seq="zxy", coordinates="right")
        elif self.nav_frame == "NED":
            maxtrix = right_hand_rule.euler_z_rotation(math.radians(90))
            new_rpy = maxtrix @ self.rpy
            roll = new_rpy.tolist()[0][0]
            pitch = new_rpy.tolist()[1][0]
            yaw = new_rpy.tolist()[2][0]
            DCM = eul2dcm(roll, pitch, yaw, seq="zyx", coordinates="right")
        new_point = DCM @ point
        return new_point
