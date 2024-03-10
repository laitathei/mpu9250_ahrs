import pygame
import numpy as np
from wireframe import wireframe
from operator import itemgetter
from mpu9250 import MPU9250
import time

class imu_viewer():
    def __init__(self, width, height, hz):
        self.width = width
        self.height = height
        self.hz = hz
        self.screen = pygame.display.set_mode((width, height))
        self.update_rate = pygame.time.Clock()
        pygame.font.init()
        self.font_size = 15
        self.font = pygame.font.SysFont('arial', self.font_size)
        # self.font = pygame.font.SysFont('Comic Sans MS', 15)
        self.background_color = (0, 0, 0)
        pygame.display.set_caption('IMU attitude display with euler angle and quaternion')

    def run(self, imu):
        self.wireframe = wireframe(imu.nav_frame)
        self.wireframe.initialize_cube(2,-2,3,-3,0.1,-0.1)
        """ Create a pygame screen until it is closed. """
        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                self.update_rate.tick(self.hz)
                self.display(imu.acc, imu.gyr, imu.mag, imu.euler, imu.quaternion, imu.body_frame, imu.nav_frame)
                pygame.display.update()
                # pygame.display.flip()

        except KeyboardInterrupt:
            print('interrupted!')

    def display(self, acc, gyr, mag, euler, quaternion, body, nav):
        # euler = np.array([[-45],[-45],[0]])
        self.screen.fill(self.background_color)
        self.wireframe.update_attitude(euler)

        ax = round(acc[0][0],2)
        ay = round(acc[1][0],2)
        az = round(acc[2][0],2)

        gx = round(gyr[0][0],2)
        gy = round(gyr[1][0],2)
        gz = round(gyr[2][0],2)

        mx = round(mag[0][0],2)
        my = round(mag[1][0],2)
        mz = round(mag[2][0],2)

        roll = round(euler[0][0],2)
        pitch = round(euler[1][0],2)
        yaw = round(euler[2][0],2)

        w = round(quaternion[0][0],2)
        x = round(quaternion[1][0],2)
        y = round(quaternion[2][0],2)
        z = round(quaternion[3][0],2)

        # Display acceleration
        self.message_display("Acc: ", self.width * 0, self.font_size * 0, (255, 255, 255))
        self.message_display("ax: {}".format(ax), self.width * 0, self.font_size * 1,(255, 255, 255))
        self.message_display("ay: {}".format(ay), self.width * 0, self.font_size * 2, (255, 255, 255))
        self.message_display("az: {}".format(az), self.width * 0, self.font_size * 3, (255, 255, 255))

        # Display angular velocity
        self.message_display("Gyro: ", self.width * 0.1, self.font_size * 0, (255, 255, 255))
        self.message_display("gx: {}".format(gx), self.width * 0.1, self.font_size * 1,(255, 255, 255))
        self.message_display("gy: {}".format(gy), self.width * 0.1, self.font_size * 2, (255, 255, 255))
        self.message_display("gz: {}".format(gz), self.width * 0.1, self.font_size * 3, (255, 255, 255))

        # Display magnetic field
        self.message_display("Mag: ", self.width * 0.2, self.font_size * 0, (255, 255, 255))
        self.message_display("mx: {}".format(mx), self.width * 0.2, self.font_size * 1,(255, 255, 255))
        self.message_display("my: {}".format(my), self.width * 0.2, self.font_size * 2, (255, 255, 255))
        self.message_display("mz: {}".format(mz), self.width * 0.2, self.font_size * 3, (255, 255, 255))

        # Display body and navigation frame setting
        self.message_display("Body frame: {}".format(body), self.width * 0.3, self.font_size * 0, (255, 255, 255))
        self.message_display("Navigation frame: {}".format(nav), self.width * 0.3, self.font_size * 1, (255, 255, 255))

        # Display quaternion
        self.message_display("Quaternion: ", self.width * 0.75, self.font_size * 0, (255, 255, 255))
        self.message_display("w: {}".format(w), self.width * 0.75, self.font_size * 1, (255, 255, 255))
        self.message_display("x: {}".format(x), self.width * 0.75, self.font_size * 2, (255, 255, 255))
        self.message_display("y: {}".format(y), self.width * 0.75, self.font_size * 3, (255, 255, 255))
        self.message_display("z: {}".format(z), self.width * 0.75, self.font_size * 4, (255, 255, 255))

        # Display euler angle
        self.message_display("Euler Angles (Degree): ", self.width * 0.85, self.font_size * 0, (255, 255, 255))
        self.message_display("Roll: {} ({})".format(roll, "X-axis"), self.width * 0.85, self.font_size * 1,(255, 255, 255))
        self.message_display("Pitch: {} ({})".format(pitch, "Y-axis"), self.width * 0.85, self.font_size * 2, (255, 255, 255))
        self.message_display("Yaw: {} ({})".format(yaw, "Z-axis"), self.width * 0.85, self.font_size * 3, (255, 255, 255))

        # Display observation message
        self.message_display("Please observe the imu from top view", self.width * 0.4, self.height * 0.95, (255, 255, 255))

        # Transform vertices to perspective view
        viewer_vertices = []
        viewer_depth = []
        for vertice in self.wireframe.vertices:
            point = np.array([[vertice.x],[vertice.y],[vertice.z]])
            new_point = self.wireframe.rotate_point(point)
            viewer_vertices.append(self.projectOthorgraphic(new_point.tolist()[0][0], new_point.tolist()[1][0], 70, imu.nav_frame))
            viewer_depth.append(new_point.tolist()[2][0])

        # Calculate the average Z values of each face.
        avg_z = []
        for face in self.wireframe.faces:
            z = (viewer_depth[face.vertice_indexs[0]] + viewer_depth[face.vertice_indexs[1]] +
                 viewer_depth[face.vertice_indexs[2]] + viewer_depth[face.vertice_indexs[3]]) / 4.0
            avg_z.append(z)

        # Draw the faces using the Painter's algorithm:
        for idx, val in sorted(enumerate(avg_z), key=itemgetter(1)): # sort the list according to avg_z
            face = self.wireframe.faces[idx]
            pointList = [viewer_vertices[face.vertice_indexs[0]],
                         viewer_vertices[face.vertice_indexs[1]],
                         viewer_vertices[face.vertice_indexs[2]],
                         viewer_vertices[face.vertice_indexs[3]]]
            pygame.draw.polygon(self.screen, face.color, pointList)

    def message_display(self, text, x, y, text_color):
        text_surface = self.font.render(text, True, text_color, self.background_color)
        text_rect = text_surface.get_rect()
        text_rect.topleft = (x, y)
        self.screen.blit(text_surface, text_rect)

    # Normal Projection
    def projectOthorgraphic(self, x, y, scaling_constant, nav_frame):
        xProjected = round(x * scaling_constant + self.width / 2)
        yProjected = round(y * scaling_constant + self.height / 2)
        return xProjected, yProjected

if __name__ == '__main__':
    window_width = 1080
    window_height = 720
    window_hz = 100
    nav_frame = "ENU" # ENU/NED
    axis = 9
    imu = MPU9250(nav_frame, axis, window_hz)
    imu.start_thread()
    viewer = imu_viewer(window_width, window_height, window_hz)
    viewer.run(imu)