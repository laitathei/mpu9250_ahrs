#!/usr/bin/python3
import smbus
import math
import time
import numpy as np
import yaml
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
    :param bool calibration: calibrate gyroscope and accelerometer
    """
    def __init__(self, bus, address, nav_frame="ENU", hz=100, calibration=False):
        # I2C connection parameter
        self.bus = bus
        self.address = address

        # Accelerometer and gyroscope parameter
        self.accel_bias = np.zeros((3,1))
        self.accel_scale = np.zeros((3,1))
        self.accel_misalignment = np.zeros((6,1))
        self.gyro_bias = np.zeros((3,1))
        self.gyro_scale = np.zeros((3,1))
        self.gyro_misalignment = np.zeros((6,1))
        self.calibration = calibration

        # driver parameter
        self.nav_frame = nav_frame # original navigation frame of MPU6500 is ENU
        self.hz = hz

        # Load old config from yaml file
        if self.calibration == False: 
            f = open("config.yaml", "r")
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            gyro_bias = ["gx_bias","gy_bias","gz_bias"]
            gyro_scale = ["gx_scale","gy_scale","gz_scale"]
            # gyro_misalignment = ["gyro_xy_mis","gyro_xz_mis","gyro_yx_mis","gyro_yz_mis","gyro_zx_mis","gyro_zy_mis"]
            accel_bias = ["ax_bias","ay_bias","az_bias"]
            accel_scale = ["ax_scale","ay_scale","az_scale"]
            # accel_misalignment = ["accel_xy_mis","accel_xz_mis","accel_yx_mis","accel_yz_mis","accel_zx_mis","accel_zy_mis"]
            for i, element in enumerate(gyro_bias):
                self.gyro_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(gyro_scale):
                self.gyro_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(gyro_misalignment):
            #     self.gyro_misalignment[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(accel_bias):
                self.accel_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(accel_scale):
                self.accel_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(accel_misalignment):
            #     self.accel_misalignment[i][0] = self.config[nav_frame][element]

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
        # config accelerometer full scale
        if accel_parameter == ACCEL_FS_SEL_2G:
            self.accel_fs = 2.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_4G:
            self.accel_fs = 4.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_8G:
            self.accel_fs = 8.0/32768.0
        elif accel_parameter == ACCEL_FS_SEL_16G:
            self.accel_fs = 16.0/32768.0
        else:
            raise ValueError("Wrong accel config parameter")

        # config gyroscope full scale
        if gyro_parameter == GYRO_FS_SEL_250DPS:
            self.gyro_fs = 250.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_500DPS:
            self.gyro_fs = 500.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_1000DPS:
            self.gyro_fs = 1000.0/32768.0
        elif gyro_parameter == GYRO_FS_SEL_2000DPS:
            self.gyro_fs = 2000.0/32768.0
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
        self.bus.write_byte_data(self.address, ACCEL_CONFIG_2, 0x06) # Set accel digital high-pass filter
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
        Calculate the gyroscope bias to calibrate the gyroscope

        :param int s: time for calibration
        :returns: 
            - gyro_scale (ndarray) - 3-axis gyroscope scale
            - gyro_bias (ndarray) - 3-axis gyroscope bias
            - gyro_misalignment (ndarray) - 3-axis gyroscope misalignment
        """
        if s > 0 and self.calibration == True:
            print('Start gyroscope calibration - Do not move the IMU for {}s'.format(s))
            gyro_bias = np.zeros((3,1))

            for i in range(s*self.hz):
                gx, gy, gz = self.get_gyro()
                gyro_bias += np.array([[gx],[gy],[gz]])
                time.sleep(1/self.hz)
            
            # Calculate bias
            self.gyro_bias = gyro_bias/(s*self.hz)

            # calculate scale
            self.gyro_scale = np.array([[1],[1],[1]])

            # calculate misalignment
            self.gyro_misalignment = np.array([[0],[0],[0],[0],[0],[0]])

            print("Finish gyroscope calibration")
        return self.gyro_scale, self.gyro_bias, self.gyro_misalignment

    def accel_calibration(self, s: int):
        """
        Calculate the accelerometer bias, scale, misalignment with six calibration measurements
        Using least square method to solve the error

        :param int s: time for calibration
        :returns: 
            - accel_scale (ndarray) - 3-axis accelerometer scale
            - accel_bias (ndarray) - 3-axis accelerometer bias
            - accel_misalignment (ndarray) - 3-axis accelerometer misalignment
        .. Reference
        .. [1] `Accelerometer calibration <https://zhuanlan.zhihu.com/p/296381805>`
        .. [2] 'Least square prove <https://zhuanlan.zhihu.com/p/87582571>'
        """
        if s > 0 and self.calibration == True:
            order = ["x","y","z","-x","-y","-z"]
            calibration = []
            for i in range(6):
                input('Place IMU {} axis ({}) pointing downward and do not move the IMU for {}s'.format(order[i], self.nav_frame, s))
                total_accel = np.zeros((3))

                for j in range(s*self.hz):
                    ax, ay, az = self.get_accel()
                    total_accel += np.array([ax, ay, az])
                    time.sleep(1/self.hz)
                avg_accel = total_accel/(s*self.hz)
                calibration.append(avg_accel.tolist())
            calibration = np.array(calibration)
            calibration = np.append(calibration,np.ones((6,1))*-1,axis=1)
            positive = np.diag(np.full(3,g))
            negative = np.diag(np.full(3,-g))
            if self.nav_frame == "ENU":
                target = np.vstack((negative,positive))
            elif self.nav_frame == "NED":
                target = np.vstack((positive,negative))
            error_matrix = np.linalg.inv(calibration.T @ calibration) @ calibration.T @ target

            # calculate bias
            x_bias = error_matrix[3][0]
            y_bias = error_matrix[3][1]
            z_bias = error_matrix[3][2]
            self.accel_bias = np.array([[x_bias],[y_bias],[z_bias]])

            # calculate scale
            x_scale = error_matrix[0][0]
            y_scale = error_matrix[1][1]
            z_scale = error_matrix[2][2]
            self.accel_scale = np.array([[x_scale],[y_scale],[z_scale]])

            # calculate misalignment
            xy_mis = error_matrix[0][1]
            xz_mis = error_matrix[0][2]
            yx_mis = error_matrix[1][0]
            yz_mis = error_matrix[1][2]
            zx_mis = error_matrix[2][0]
            zy_mis = error_matrix[2][1]
            self.accel_misalignment = np.array([[xy_mis],[xz_mis],[yx_mis],[yz_mis],[zx_mis],[zy_mis]])
            print("Finish accelerometer calibration")
        return self.accel_scale, self.accel_bias, self.accel_misalignment

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
        az = +9.80665 m/s^2 when under side pointing downward \n
    
        :returns: 
            - ax (float) - x-axis accelerometer data in m/s^2
            - ay (float) - y-axis accelerometer data in m/s^2
            - az (float) - z-axis accelerometer data in m/s^2
        """
        try:
            ax = self.read_raw_data(ACCEL_XOUT_H, ACCEL_XOUT_L)*self.accel_fs
            ay = self.read_raw_data(ACCEL_YOUT_H, ACCEL_YOUT_L)*self.accel_fs
            az = self.read_raw_data(ACCEL_ZOUT_H, ACCEL_ZOUT_L)*self.accel_fs
        except:
            raise ConnectionError("I2C Connection Failure")
            
        # convert to m/s^2
        ax = ax*g
        ay = ay*g
        az = az*g

        # convert to NED frame
        if self.nav_frame == "NED":
            ax = ax*-1
            ay = ay*-1
            az = az*-1
            ax, ay, az = ENU2NED(ax, ay, az)

        # accelerometer model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.accel_scale[0][0]
        y_scale = self.accel_scale[1][0]
        z_scale = self.accel_scale[2][0]
        x_bias = self.accel_bias[0][0]
        y_bias = self.accel_bias[1][0]
        z_bias = self.accel_bias[2][0]
        xy_mis = self.accel_misalignment[0][0]
        xz_mis = self.accel_misalignment[1][0]
        yx_mis = self.accel_misalignment[2][0]
        yz_mis = self.accel_misalignment[3][0]
        zx_mis = self.accel_misalignment[4][0]
        zy_mis = self.accel_misalignment[5][0]
        if self.calibration == False:
            ax = (x_scale * ax + xy_mis * ay + xz_mis * az) - x_bias
            ay = (yx_mis * ax + y_scale * ay + yz_mis * az) - y_bias
            az = (zx_mis * ax + zy_mis * ay + z_scale * az) - z_bias
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
            gx = self.read_raw_data(GYRO_XOUT_H, GYRO_XOUT_L)*self.gyro_fs*math.pi/180
            gy = self.read_raw_data(GYRO_YOUT_H, GYRO_YOUT_L)*self.gyro_fs*math.pi/180
            gz = self.read_raw_data(GYRO_ZOUT_H, GYRO_ZOUT_L)*self.gyro_fs*math.pi/180
        except:
            raise ConnectionError("I2C Connection Failure")

        # convert to NED frame
        if self.nav_frame == "NED":
            gx, gy, gz = ENU2NED(gx, gy, gz)

        # gyroscope model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.gyro_scale[0][0]
        y_scale = self.gyro_scale[1][0]
        z_scale = self.gyro_scale[2][0]
        x_bias = self.gyro_bias[0][0]
        y_bias = self.gyro_bias[1][0]
        z_bias = self.gyro_bias[2][0]
        xy_mis = self.gyro_misalignment[0][0]
        xz_mis = self.gyro_misalignment[1][0]
        yx_mis = self.gyro_misalignment[2][0]
        yz_mis = self.gyro_misalignment[3][0]
        zx_mis = self.gyro_misalignment[4][0]
        zy_mis = self.gyro_misalignment[5][0]  
        if self.calibration == False:
            gx = (x_scale * gx + xy_mis * gy + xz_mis * gz) - x_bias
            gy = (yx_mis * gx + y_scale * gy + yz_mis * gz) - y_bias
            gz = (zx_mis * gx + zy_mis * gy + z_scale * gz) - z_bias

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
