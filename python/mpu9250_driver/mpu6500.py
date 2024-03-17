#!/usr/bin/python3
import smbus
import math
import time
import numpy as np
from utils.transformation import ENU2NED

SMPLRT_DIV = 0x19
BYPASS_ENABLE = 0x37
GYRO_CONFIG_2 = 0x1a
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

# standard acceleration of gravity
g = 9.80665

class MPU6500():
    """
    MPU6500 I2C driver for acquire accelerometer and gyroscope data

    :param SMBus bus: device I2C port
    :param int address: MPU6500 I2C address
    :param str nav_frame: navigation frame
    :param int hz: IMU frequency
    """
    def __init__(self, bus, address, nav_frame="ENU", hz=100):
        # I2C connection parameter
        self.bus = bus
        self.address = address

        # Accelerometer and gyroscope parameter
        self.accel_offset = np.zeros((3,1))
        self.gyro_offset = np.zeros((3,1))

        # driver parameter
        self.nav_frame = nav_frame # original navigation frame of MPU6500 is ENU
        self.hz = hz

        # Check parameter
        if (self.nav_frame != "ENU") and (self.nav_frame != "NED"):
            raise ValueError("Navigation frame should be either ENU or NED")
            
    def control_accel_gyro(self, ax=True, ay=True, az=True, gx=True, gy=True, gz=True):
        """
        Whether to enable accelerometer and gyroscope xyz axis data

        :param bool ax: enable x axis accelerometer data
        :param bool ay: enable y axis accelerometer data
        :param bool az: enable z axis accelerometer data
        :param bool gx: enable x axis gyroscope data
        :param bool gy: enable y axis gyroscope data
        :param bool gz: enable z axis gyroscope data
        """
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

    def who_am_i(self):
        """
        Check MPU6500 WHOAMI register value
        """
        value = hex(self.bus.read_byte_data(self.address, WHO_AM_I))
        print("The register value is {}".format(value))
        if value == "0x71":
            print("It is MPU6500 default value")
        else:
            print("It is not MPU6500 default value")
            raise RuntimeError("MPU6500 not found")

    def config_MPU6500(self, accel_parameter, gyro_parameter):
        """
        Config MPU6500 accelerometer and gyroscope scale with internal DMP

        :param int accel_parameter: accelerometer configuration register value
        :param int gyro_parameter: gyroscope configuration register value
        """
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

        # Write byte data to MPU6500 gyroscope and accelerometer configuration register
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, accel_parameter)
        self.bus.write_byte_data(self.address, GYRO_CONFIG, gyro_parameter)

        # MPU6500 and AK8963 share the same I2C
        # MPU6500 is master
        # AK8963 is slave
        value = 0x02
        self.bus.write_byte_data(self.address, BYPASS_ENABLE, value)

        # ACCEL_CONFIG_2   | Bandwidth | Delay   
        # 0x00             | 250Hz     | 1.88ms
        # 0x01             | 184Hz     | 1.88ms  
        # 0x02             | 92Hz      | 2.88ms  
        # 0x03             | 41Hz      | 4.88ms  
        # 0x04             | 20Hz      | 8.87ms  
        # 0x05             | 10Hz      | 16.83ms
        # 0x06             | 5Hz       | 32.48ms
        # 0x07             | 3600Hz    | 1.38ms

        # GYRO_CONFIG_2   | Bandwidth | Delay   | Fs   | 
        # 0x00            | 250Hz     | 0.97ms  | 8kHz | 
        # 0x01            | 184Hz     | 2.9ms   | 1kHz | 
        # 0x02            | 92Hz      | 3.9ms   | 1kHz | 
        # 0x03            | 41Hz      | 5.9ms   | 1kHz | 
        # 0x04            | 20Hz      | 9.9ms   | 1kHz | 
        # 0x05            | 10Hz      | 17.85ms | 1kHz | 
        # 0x06            | 5Hz       | 33.48ms | 1kHz | 
        # 0x07            | 3600Hz    | 0.17ms  | 8kHz | 
        self.bus.write_byte_data(self.address, ACCEL_CONFIG_2, 0x06) # Set acc digital high-pass filter
        self.bus.write_byte_data(self.address, GYRO_CONFIG_2, 0x06) # Set gyro digital low-pass filter
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x00) # Set sample rate to 1 kHz

    def read_raw_data(self, high_register, low_register):
        """
        Access the high and low registers of the accelerometer and gyroscope to calculate their values into int16 format

        :param int high_register: high registers of the accelerometer or gyroscope
        :param int low_register: low registers of the accelerometer or gyroscope

        :returns: 
            - signed_value (int) - sensor value in int16 format
        """
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

    def gyro_calibration(self, s: int):
        """
        Calculate the gyroscope offset to calibrate the gyroscope

        :param int s: time for calibration
        """
        if s > 0:
            print('Start gyroscope calibration - Do not move the IMU for {}s'.format(s))
            gyro_offset = np.zeros((3,1))

            for i in range(s*self.hz):
                gx, gy, gz = self.get_gyro()
                gyro_offset += np.array([[gx],[gy],[gz]])
                time.sleep(1/self.hz)
            self.gyro_offset = gyro_offset/(s*self.hz)
            print("Finish gyroscope calibration")

    def get_accel(self):
        """
        MPU6500 accelerometer data in Earth's reference (m/s^2)
        Accelerometer channel is negative when pointing up and aligned against gravity

        ENU: \n
        Gravity is defined as negative when pointing upward \n
        ax = +9.80665 m/s^2 when the right hand side pointing upward \n
        ay = +9.80665 m/s^2 when front side pointing upward \n
        az = +9.80665 m/s^2 when upper side pointing upward \n

        NED:
        Gravity is defined as negative when pointing downward \n
        ax = +9.80665 m/s^2 when front side pointing downward \n
        ay = +9.80665 m/s^2 when the right hand side pointing downward \n
        az = +9.80665 m/s^2 when under side pointing upward \n
    
        :returns: 
            - ax (float) - x-axis accelerometer data in m/s^2
            - ay (float) - y-axis accelerometer data in m/s^2
            - az (float) - z-axis accelerometer data in m/s^2
        """
        try:
            ax = self.read_raw_data(ACCEL_XOUT_H, ACCEL_XOUT_L)*self.accel_scale
            ay = self.read_raw_data(ACCEL_YOUT_H, ACCEL_YOUT_L)*self.accel_scale
            az = self.read_raw_data(ACCEL_ZOUT_H, ACCEL_ZOUT_L)*self.accel_scale
        except:
            raise ConnectionError("I2C Connection Failure")
            
        accel = np.array([[ax],[ay],[az]])
        accel = accel-self.accel_offset

        # convert to m/s^2
        ax = accel[0][0]*g
        ay = accel[1][0]*g
        az = accel[2][0]*g

        if self.nav_frame == "NED":
            ax = ax*-1
            ay = ay*-1
            az = az*-1
            ax, ay, az = ENU2NED(ax, ay, az)
        return ax, ay, az

    def get_gyro(self):
        """
        MPU6500 gyroscope data in right hand coordinates (rad/s)

        ENU: \n
        gx is positive when rotate clockwise along x-axis \n
        gy is positive when rotate clockwise along y-axis \n
        gz is positive when rotate anti-clockwise along z-axis \n

        NED: \n
        gx is positive when rotate clockwise along x-axis \n
        gy is positive when rotate clockwise along y-axis \n
        gz is positive when rotate clockwise along z-axis \n

        :returns: 
            - gx (float) - x-axis gyroscope data in rad/s
            - gy (float) - y-axis gyroscope data in rad/s
            - gz (float) - z-axis gyroscope data in rad/s
        """
        try:
            gx = self.read_raw_data(GYRO_XOUT_H, GYRO_XOUT_L)*self.gyro_scale
            gy = self.read_raw_data(GYRO_YOUT_H, GYRO_YOUT_L)*self.gyro_scale
            gz = self.read_raw_data(GYRO_ZOUT_H, GYRO_ZOUT_L)*self.gyro_scale
        except:
            raise ConnectionError("I2C Connection Failure")

        gyro = np.array([[gx],[gy],[gz]])
        gyro = gyro-self.gyro_offset

        # convert to rad/s
        gx = gyro[0][0]*math.pi/180
        gy = gyro[1][0]*math.pi/180
        gz = gyro[2][0]*math.pi/180

        if self.nav_frame == "NED":
            gx, gy, gz = ENU2NED(gx, gy, gz)
        return gx, gy, gz

    def get_temp(self):
        """
        MPU6500 temperature data

        :returns: 
            - temp (float) - temperature data in °C
        """
        temp = self.read_raw_data(TEMP_OUT_H, TEMP_OUT_L)
        temp = ((temp - ROOM_TEMP_OFFSET)/TEMP_SENSITIVITY) + 21.0
        return temp
