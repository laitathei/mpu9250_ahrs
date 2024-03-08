#!/usr/bin/python3
import smbus
import math
import time
import numpy as np
from progress.bar import Bar
from .transformation import NED2ENU, ENU2NED

# Reference Github
# https://github.com/adityanarayanan03/MPU9250/tree/master
# https://github.com/kevinmcaleer/mpu9250/tree/main

# << 8 (add 8 zeros in the back of hex number)
# >> 8 (add 8 zeros in the front of hex number)

SMPLRT_DIV = 0x19
DLPF_CFG = 0x1A
SMPLRT_DIV = 0x19
BYPASS_ENABLE = 0x37
GYRO_CONFIG = 0x1b
ACCEL_CONFIG = 0x1c
ACCEL_CONFIG_2 = 0x1d
ACCEL_XOUT_H = 0x3b
ACCEL_XOUT_L = 0x3c
ACCEL_YOUT_H = 0x3d
ACCEL_YOUT_L = 0x3e
ACCEL_ZOUT_H = 0x3f
ACCEL_ZOUT_L = 0x40
TEMP_OUT_H = 0x41
TEMP_OUT_L = 0x42
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c
WHO_AM_I = 0x75 # default return 0x71

ACCEL_FS_SEL_2G = 0 # 0b00000000
ACCEL_FS_SEL_4G = 8 # 0b00001000
ACCEL_FS_SEL_8G = 16 # 0b00010000
ACCEL_FS_SEL_16G = 24 # 0b00011000

GYRO_FS_SEL_250DPS = 0 # 0b00000000
GYRO_FS_SEL_500DPS = 8 # 0b00001000
GYRO_FS_SEL_1000DPS = 16 # 0b00010000
GYRO_FS_SEL_2000DPS = 24 # 0b00011000

# From MPU9250 datasheet 3.4.2
TEMP_SENSITIVITY = 333.87
ROOM_TEMP_OFFSET = 0

class MPU6500():
    def __init__(self, bus, address, nav_frame="ENU"):
        # original navigation frame of MPU6500 is ENU
        self.bus = bus
        self.address = address
        self.accel_offset = np.zeros((3,1))
        self.gyro_offset = np.zeros((3,1))
        self.nav_frame = nav_frame
        
    def control_accel_gyro(self, ax=True, ay=True, az=True, gx=True, gy=True, gz=True):
        value = 0 # 0b00000000
        if ax == False:
            value += 4 # 0b10000000
        if ay == False:
            value += 8 # 0b01000000
        if az == False:
            value += 16 # 0b00100000
        if gx == False:
            value += 32 # 0b10000000
        if gy == False:
            value += 64 # 0b01000000
        if gz == False:
            value += 128 # 0b00100000
        self.bus.write_byte_data(self.address, PWR_MGMT_2, value)

    def check_address(self):
        address_list = []
        for device in range(128):
            try:
                self.bus.read_byte(device)
                address_list.append(hex(device))
            except:
                pass
        return address_list

    def who_am_i(self):
        value = hex(self.bus.read_byte_data(self.address, WHO_AM_I))
        print("The register value is {}".format(value))
        if value == "0x71":
            print("It is MPU6500 default value")
        else:
            print("It is not MPU6500 default value")
            raise RuntimeError("MPU6500 not found")

    def config_MPU6500(self, accel_parameter, gyro_parameter):
        # 32768 is the positive range of int16
        if accel_parameter == ACCEL_FS_SEL_2G:
            self.accel_scale = 2.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_4G:
            self.accel_scale = 4.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_8G:
            self.accel_scale = 8.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_16G:
            self.accel_scale = 16.0/32768.0
        else:
            raise ValueError("Wrong accel config parameter")

        if gyro_parameter == GYRO_FS_SEL_250DPS:
            self.gyro_scale = 250.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_500DPS:
            self.gyro_scale = 500.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_1000DPS:
            self.gyro_scale = 1000.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_2000DPS:
            self.gyro_scale = 2000.0/32768.0
        else:
            raise ValueError("Wrong gyro config parameter")

        # Write byte data to MPU6500 Gyroscope and Accelerometer Configuration register
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, accel_parameter)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, gyro_parameter)

        # MPU6500 and AK8963 share the same I2C
        # MPU6500 is master
        # AK8963 is slave
        value = 0x02
        self.bus.write_byte_data(self.address, BYPASS_ENABLE, value)

        # DLPF_CFG | Bandwidth | Delay   | Fs   | 
        # 0x00     | 250Hz     | 0.97ms  | 8kHz | 
        # 0x01     | 184Hz     | 2.9ms   | 1kHz | 
        # 0x02     | 92Hz      | 3.9ms   | 1kHz | 
        # 0x03     | 41Hz      | 5.9ms   | 1kHz | 
        # 0x04     | 20Hz      | 9.9ms   | 1kHz | 
        # 0x05     | 10Hz      | 17.85ms | 1kHz | 
        # 0x06     | 5Hz       | 33.48ms | 1kHz | 
        # 0x07     | 3600Hz    | 0.17ms  | 8kHz | 
        self.bus.write_byte_data(self.address, DLPF_CFG, 0x07) # Set gyro digital low-pass filter
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x00) # Set sample rate to 1 kHz

    def read_raw_data(self, high_register, low_register):
        high = self.bus.read_byte_data(self.address, high_register)
        low = self.bus.read_byte_data(self.address, low_register)

        # Megre higher bytes and lower bytes data
        unsigned_value = (high << 8) + low

        # Calculate the unsigned int16 range to signed int16 range
        if (unsigned_value >= 32768) and (unsigned_value < 65536):
            signed_value = unsigned_value - 65536
        elif (unsigned_value >= 0) and (unsigned_value < 32768):
            signed_value = unsigned_value
        return signed_value

    def read_register(self, register):
        value = self.bus.read_byte_data(self.address, register)
        return value

    def gyro_calibration(self, count, hz):
        total_gx = 0
        total_gy = 0
        total_gz = 0
        interval = 1/hz
        last = time.time()
        n = count
        while count > 0:
            next = last + interval
            time.sleep(abs(next - time.time()))  # it's ok to sleep negative time
            gx, gy, gz = self.get_gyro()
            total_gx += gx
            total_gy += gy
            total_gz += gz
            if count % 100 == 0:
                print("Calibrating gyroscope... {}%".format((count/n)*100))
            count = count - 1
            last = time.time()
        self.gyro_offset[0][0] = total_gx/n
        self.gyro_offset[1][0] = total_gy/n
        self.gyro_offset[2][0] = total_gz/n

    def get_accel(self):
        # MPU6500 accelerometer data in Earth’s reference (g)
        try:
            ax = self.read_raw_data(ACCEL_XOUT_H, ACCEL_XOUT_L)*self.accel_scale
            ay = self.read_raw_data(ACCEL_YOUT_H, ACCEL_YOUT_L)*self.accel_scale
            az = self.read_raw_data(ACCEL_ZOUT_H, ACCEL_ZOUT_L)*self.accel_scale
        except:
            raise ConnectionError("I2C Connection Failure")
            
        # set the downward gravity is positive
        accel = np.array([[ax],[ay],[az]])
        accel = accel-self.accel_offset
        ax = accel[0][0]*-1
        ay = accel[1][0]*-1
        az = accel[2][0]*-1

        if self.nav_frame == "NED":
            ax, ay, az = ENU2NED(ax, ay, az)
        return ax, ay, az

    def get_gyro(self):
        try:
            gx = self.read_raw_data(GYRO_XOUT_H, GYRO_XOUT_L)*self.gyro_scale
            gy = self.read_raw_data(GYRO_YOUT_H, GYRO_YOUT_L)*self.gyro_scale
            gz = self.read_raw_data(GYRO_ZOUT_H, GYRO_ZOUT_L)*self.gyro_scale
        except:
            raise ConnectionError("I2C Connection Failure")

        gyro = np.array([[gx],[gy],[gz]])
        gyro = gyro-self.gyro_offset
        gx = gyro[0][0]
        gy = gyro[1][0]
        gz = gyro[2][0] 
        if self.nav_frame == "NED":
            gx, gy, gz = ENU2NED(gx, gy, gz)
        return gx, gy, gz

    def get_temp(self):
        temp = self.read_raw_data(TEMP_OUT_H, TEMP_OUT_L)
        temp = ((temp - ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY) + 21.0
        return temp
