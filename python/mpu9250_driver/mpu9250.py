#!/usr/bin/python3
from .mpu6500 import MPU6500
from .ak8963 import AK8963
from utils.transformation import acc2eul, acc2quat, accmag2eul, accmag2quat
from utils.orientation import quat2eul
from utils.filter import sliding_window
import smbus
import math
import numpy as np
from threading import Thread

class MPU9250():
    def __init__(self, nav_frame, axis, hz, s):
        """
        MPU9250 I2C driver for accessing MPU6500 and AK8963

        :param str nav_frame: navigation frame
        :param int axis: axis data
        :param float hz: IMU frequency
        :param float s: calibration time

        .. Reference
        .. [1] `MPU-9250 Product Specification <https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf>`
        .. [2] `MPU-9250 Register Map and Descriptions <https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf>`
        """
        # I2C connection parameter
        self.mpu6500_address = 0x68
        self.ak8963_address = 0x0c
        self.bus = smbus.SMBus(1)
        address_list = self.check_i2c_address()
        if len(address_list) == 0:
            raise RuntimeError("No i2c address found")
        else: 
            print("Default MPU6500 address is 0x68")
            print("Default AK8963 address is 0x0c")

        # driver parameter
        self.nav_frame = nav_frame
        self.axis = axis
        self.hz = hz
        self.dt = 1/self.hz
        self.calibration_time = s
        self.queue_size = 20
        self.window_size = 5
        self.gyro_queue = np.empty([1,3])

        # Check parameter
        if (self.axis != 6) and (self.axis != 9):
            raise ValueError("Axis must be 6 or 9")
        if self.nav_frame=="NED":
            self.body_frame = "FRD"
            self.rotation_seq = "zyx"
        elif self.nav_frame=="ENU":
            self.body_frame = "RFU"
            self.rotation_seq = "zxy"
        else:
            raise ValueError("Navigation frame should be either ENU or NED")

        # Config MPU6500
        self.mpu6500 = MPU6500(self.bus, self.mpu6500_address, self.nav_frame, self.hz)
        self.mpu6500.control_accel_gyro()
        self.mpu6500.who_am_i()
        self.mpu6500.config_MPU6500(0, 0)

        # Config AK8963
        self.ak8963 = AK8963(self.bus, self.ak8963_address, self.nav_frame, self.hz)
        self.ak8963.who_am_i()
        self.ak8963.config_AK8963(16)

        # initialization
        self.initialization()
        self.mpu6500.gyro_calibration(self.calibration_time)
        # self.mpu6500.accel_calibration(self.calibration_time)

    def check_i2c_address(self):
        """
        Search all I2C addresses and record accessible addresses

        :returns: 
            - address_list - stores a list of all accessible I2C addresses
        """
        address_list = []
        for device in range(128):
            try:
                self.bus.read_byte(device)
                address_list.append(hex(device))
            except:
                pass
        return address_list

    def get_accel(self):
        """
        MPU9250 accelerometer data in Earth's reference (m/s^2)
        Accelerometer channel is negative when pointing up and aligned against gravity

        ENU:
        Gravity is defined as negative when pointing upward
        ax = +9.80665 m/s^2 when the right hand side pointing upward
        ay = +9.80665 m/s^2 when front side pointing upward
        az = +9.80665 m/s^2 when upper side pointing upward

        NED:
        Gravity is defined as negative when pointing downward
        ax = +9.80665 m/s^2 when front side pointing downward
        ay = +9.80665 m/s^2 when the right hand side pointing downward
        az = +9.80665 m/s^2 when under side pointing upward
    
        :returns: 
            - ax - x-axis accelerometer data in m/s^2
            - ay - y-axis accelerometer data in m/s^2
            - az - z-axis accelerometer data in m/s^2
        """
        self.ax, self.ay, self.az = self.mpu6500.get_accel()
        self.acc = np.array([[self.ax],[self.ay],[self.az]])
        return self.ax, self.ay, self.az

    def get_gyro(self):
        """
        MPU9250 gyroscope data in right hand coordinates (rad/s)

        ENU:
        gx is positive when rotate clockwise along x-axis
        gy is positive when rotate clockwise along y-axis
        gz is positive when rotate anti-clockwise along z-axis

        NED:
        gx is positive when rotate clockwise along x-axis
        gy is positive when rotate clockwise along y-axis
        gz is positive when rotate clockwise along z-axis

        :returns: 
            - gx - x-axis gyroscope data in rad/s
            - gy - y-axis gyroscope data in rad/s
            - gz - z-axis gyroscope data in rad/s
        """
        self.gx, self.gy, self.gz = self.mpu6500.get_gyro()
        self.gyr = np.array([[self.gx],[self.gy],[self.gz]])

        # # Sliding window 
        # if len(self.gyro_queue) < self.queue_size:
        #     # append to the last
        #     self.gyro_queue = np.append(self.gyro_queue, self.gyr.T, axis = 0)
        # else:
        #     # remove first element
        #     self.gyro_queue = np.delete(self.gyro_queue, 0, 0)
        #     # append to the last
        #     self.gyro_queue = np.append(self.gyro_queue, self.gyr.T, axis = 0)

        # if len(self.gyro_queue) > self.window_size:
        #     sliding_window(self.gyro_queue, self.window_size)
        #     self.gx, self.gy, self.gz = self.gyro_queue[0][0], self.gyro_queue[0][1], self.gyro_queue[0][2]
        #     self.gyr = np.array([[self.gx],[self.gy],[self.gz]])
        return self.gx, self.gy, self.gz

    def get_mag(self):
        """
        MPU9250 magnetometer data in Earth's reference (µT)

        ENU:
        mx is positive when the right hand side pointing to north
        my is positive when the front side pointing to north
        mz is positive when the upper side pointing to north

        NED:
        mx is positive when the front side pointing to north
        my is positive when the right hand side pointing to north
        mz is positive when the under side pointing to north
    
        :returns: 
            - mx - x-axis magnetometer data in µT
            - my - y-axis magnetometer data in µT
            - mz - z-axis magnetometer data in µT
        """
        self.mx, self.my, self.mz = self.ak8963.get_mag()
        self.mag = np.array([[self.mx],[self.my],[self.mz]])
        return self.mx, self.my, self.mz

    def get_temp(self):
        """
        MPU9250 temperature data

        :returns: temperature data in °C
        """
        self.temp = self.mpu6500.get_temp()
        return self.temp

    def get_euler(self):
        """
        MPU9250 Euler angle

        :returns: 
            - roll - x-axis Euler angle in degree
            - pitch - y-axis Euler angle in degree
            - yaw - z-axis Euler angle in degree
        """
        if self.axis == 6:
            self.roll, self.pitch, self.yaw = acc2eul(self.ax, self.ay, self.az, nav=self.nav_frame)
        elif self.axis == 9:
            self.roll, self.pitch, self.yaw = accmag2eul(self.ax, self.ay, self.az, self.mx, self.my, self.mz, nav=self.nav_frame)
        self.roll, self.pitch, self.yaw = math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw)
        self.euler = np.array([[self.roll],[self.pitch],[self.yaw]])
        return self.roll, self.pitch, self.yaw

    def get_ahrs_euler(self):
        """
        MPU9250 Euler angle processed by AHRS

        :returns: 
            - roll - x-axis Euler angle in degree
            - pitch - y-axis Euler angle in degree
            - yaw - z-axis Euler angle in degree
        """
        self.w, self.x, self.y, self.z = self.ahrs.run(self.acc, self.gyr, self.mag, self.hz)
        self.roll, self.pitch, self.yaw = quat2eul(self.w, self.x, self.y, self.z, seq=self.rotation_seq)
        self.roll, self.pitch, self.yaw = math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw)
        self.roll, self.pitch, self.yaw = round(self.roll,2), round(self.pitch,2), round(self.yaw,2)
        self.euler = np.array([[self.roll],[self.pitch],[self.yaw]])
        return self.roll, self.pitch, self.yaw

    def get_quaternion(self):
        """
        MPU9250 Quaternion

        :returns: 
            - w - Quaternion magnitude
            - x - Quaternion X axis
            - y - Quaternion Y axis
            - z - Quaternion Z axis
        """
        if self.axis == 6:
            self.w, self.x, self.y, self.z = acc2quat(self.ax, self.ay, self.az, nav=self.nav_frame)
        elif self.axis == 9:
            self.w, self.x, self.y, self.z = accmag2quat(self.ax, self.ay, self.az, self.mx, self.my, self.mz, nav=self.nav_frame)
        self.quaternion = np.array([[self.w],[self.x],[self.y],[self.z]])
        return self.w, self.x, self.y, self.z

    def get_ahrs_quaternion(self):
        """
        MPU9250 quaternion processed by AHRS

        :returns: 
            - w - Quaternion magnitude
            - x - Quaternion X axis
            - y - Quaternion Y axis
            - z - Quaternion Z axis
        """
        self.w, self.x, self.y, self.z = self.ahrs.run(self.acc, self.gyr, self.mag, self.hz)
        self.quaternion = np.array([[self.w],[self.x],[self.y],[self.z]])
        return self.w, self.x, self.y, self.z

    def initialization(self):
        """
        MPU9250 initialization for accelerometer, gyroscope, magnetometer
        """
        self.ax, self.ay, self.az = self.get_accel()
        self.gx, self.gy, self.gz = self.get_gyro()
        self.mx, self.my, self.mz = self.get_mag()
        self.roll, self.pitch, self.yaw = self.get_euler()
        self.w, self.x, self.y, self.z = self.get_quaternion()

    def start_thread(self, ahrs=None):
        """
        MPU9250 thread to continuously acquire accelerometer, gyroscope, magnetometer, attitudes, and temperature data

        :param ahrs: ahrs object (Madgwick, Mahony, EKF, etc.)
        """
        if ahrs != None:
            self.ahrs = ahrs
            self.ahrs.init_quat(self.w, self.x, self.y, self.z)
        else:
            self.ahrs = None
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        """
        Acquire accelerometer, gyroscope, magnetometer, attitudes, and temperature data
        """
        try:
            while True:
                self.ax, self.ay, self.az = self.get_accel()
                self.gx, self.gy, self.gz = self.get_gyro()
                self.mx, self.my, self.mz = self.get_mag()
                if self.ahrs != None:
                    self.roll, self.pitch, self.yaw = self.get_ahrs_euler()
                    self.w, self.x, self.y, self.z = self.get_ahrs_quaternion()
                    # print("ahrs roll pitch yaw: ", self.roll, self.pitch, self.yaw)
                else:
                    self.roll, self.pitch, self.yaw = self.get_euler()
                    self.w, self.x, self.y, self.z = self.get_quaternion()
                    # print("raw roll pitch yaw: ", round(self.roll,2), round(self.pitch,2), round(self.yaw,2))
                # print("ax ay az: ", round(self.ax,2), round(self.ay,2), round(self.az,2))
                # print("gx gy gz: ", round(self.gx,2), round(self.gy,2), round(self.gz,2))
                # print("mx my mz: ", round(self.mx,2), round(self.my,2), round(self.mz,2))
                # print("w x y z: ", round(self.w,2), round(self.x,2), round(self.y,2), round(self.z,2))
                self.temp = self.get_temp()
        except KeyboardInterrupt:
            self.thread.join()
            print('interrupted!')