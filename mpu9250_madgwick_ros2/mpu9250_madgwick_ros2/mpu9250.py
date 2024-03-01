#!/usr/bin/python3
from .mpu6500 import MPU6500
from .ak8963 import AK8963
from .transformation import acc2eul, acc2quat, accmag2eul, accmag2quat, quat2eul
import smbus
import math

# MPU-9250 Product Specification
# https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

# MPU-9250 Register Map and Descriptions
# https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf

class MPU9250():
    def __init__(self, nav_frame):
        self.bus = smbus.SMBus(1) # /dev/i2c-1
        address_list = self.check_i2c_address()
        if len(address_list) == 0:
            raise RuntimeError("No i2c address found")
        else: 
            print("Default MPU6500 address is 0x68")
            print("Default AK8963 address is 0x0c")

        self.mpu6500_address = 0x68
        self.ak8963_address = 0x0c
        self.nav_frame = nav_frame

        # Config MPU6500
        self.mpu6500 = MPU6500(self.bus, self.mpu6500_address, self.nav_frame)
        self.mpu6500.who_am_i()
        self.mpu6500.config_MPU6500(0, 0)
        
        # Config AK8963
        self.ak8963 = AK8963(self.bus, self.ak8963_address, self.nav_frame)
        self.ak8963.who_am_i()
        self.ak8963.config_AK8963(16)

    def check_i2c_address(self):
        address_list = []
        for device in range(128):
            try:
                self.bus.read_byte(device)
                address_list.append(hex(device))
            except:
                pass
        return address_list

    def get_accel(self):
        self.ax, self.ay, self.az = self.mpu6500.get_accel()
        return self.ax, self.ay, self.az

    def get_gyro(self):
        self.gx, self.gy, self.gz = self.mpu6500.get_gyro()
        return self.gx, self.gy, self.gz

    def get_mag(self):
        self.mx, self.my, self.mz = self.ak8963.get_mag()
        return self.mx, self.my, self.mz

    def get_temp(self):
        self.temp = self.mpu6500.get_temp()
        return self.temp

    def get_rpy(self, axis=9):
        if axis == 6:
            self.roll, self.pitch, self.yaw = acc2eul(self.ax, self.ay, self.az, nav=self.nav_frame)
        elif axis == 9:
            self.roll, self.pitch, self.yaw = accmag2eul(self.ax, self.ay, self.az, self.mx, self.my, self.mz, nav=self.nav_frame)
        else:
            raise ValueError("Axis must be 6 or 9")
        self.roll, self.pitch, self.yaw = math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw)
        return self.roll, self.pitch, self.yaw
