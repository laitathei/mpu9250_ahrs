#!/usr/bin/python3
import smbus
import time
import yaml
import numpy as np
from utils.transformation import NED2ENU

WIA = 0x00
INFO = 0x01
ST1 = 0x02
HXL = 0x03
HXH = 0x04
HYL = 0x05
HYH = 0x06
HZL = 0x07
HZH = 0x08
ST2 = 0x09
CNTL = 0x0A
ASAX = 0x10
ASAY = 0x11
ASAZ = 0x12

class AK8963():
    """
    AK8963 I2C driver for acquire magnetometer data

    :param SMBus bus: device I2C port
    :param int address: AK8963 I2C address
    :param str nav_frame: navigation frame
    :param int hz: IMU frequency
    """
    def __init__(self, bus, address, nav_frame="NED", hz=100, calibration=False):
        # I2C connection parameter
        self.bus = bus
        self.address = address

        # Magnetometer parameter
        self.mag_bias = np.zeros((3,1))
        self.mag_scale = np.zeros((3,1))
        self.mag_misalignment = np.zeros((6,1))
        self.mag_strength = 0
        self.calibration = calibration

        # driver parameter
        self.nav_frame = nav_frame # original navigation frame of AK8963 is NED
        self.hz = hz

        # Load old config from yaml file
        if self.calibration == False: 
            f = open("config.yaml", "r")
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            mag_bias = ["mx_bias","my_bias","mz_bias"]
            mag_scale = ["mx_scale","my_scale","mz_scale"]
            # mag_misalignment = ["mag_xy_mis","mag_xz_mis","mag_yx_mis","mag_yz_mis","mag_zx_mis","mag_zy_mis"]
            for i, element in enumerate(mag_bias):
                self.mag_bias[i][0] = self.config[nav_frame][element]
            for i, element in enumerate(mag_scale):
                self.mag_scale[i][0] = self.config[nav_frame][element]
            # for i, element in enumerate(mag_misalignment):
            #     self.mag_misalignment[i][0] = self.config[nav_frame][element]

        # Check parameter
        if (self.nav_frame != "ENU") and (self.nav_frame != "NED"):
            raise ValueError("Navigation frame should be either ENU or NED")
            
    def who_am_i(self):
        """
        Check AK8963 WHOAMI register value
        """
        value = hex(self.bus.read_byte_data(self.address, WIA))
        print("The register value is {}".format(value))
        if value == "0x48":
            print("It is AK8963 default value")
        else:
            print("It is not AK8963 default value")
            raise RuntimeError("AK8963 not found")

    def config_AK8963(self, bit):
        """
        Config AK8963 magnetometer mode

        :param int bit: magnetometer configuration register value
        """
        self.set_mode("fuse rom access", 16)
        self.get_adjust_mag()
        self.set_mode("power down", 16)
        self.set_mode("continuous measure 2", 16)
        self.get_status()

    def get_status(self):
        """
        Check AK8963 magnetometer status
        """
        ST1_value = self.bus.read_byte_data(self.address, ST1)
        bit_0 = ST1_value & int("00000001", 2)
        bit_1 = ST1_value & int("00000010", 2)
        if bit_0 == 0:
            print("Ready in measurement data register or ST2 register")
        elif bit_0 == 1:
            print("Ready in single measurement mode or self-test mode")
        else:
            raise ValueError("AK8963 status 1 register bit 0 error")
        
        if bit_1 == 0:
            print("Ready in measurement data register or ST2 register")
        elif bit_1 == 2:
            print("Data overrun")
        else:
            raise ValueError("AK8963 status 1 register bit 1 error")

    def get_adjust_mag(self):
        """
        AK8963 sensitivity adjustment value for xyz axis
        """
        print("Read sensitivity adjustment value")
        asax = self.bus.read_byte_data(self.address, ASAX)
        asay = self.bus.read_byte_data(self.address, ASAY)
        asaz = self.bus.read_byte_data(self.address, ASAZ)

        self.adjustment_x = (((asax-128)*0.5/128)+1)
        self.adjustment_y = (((asay-128)*0.5/128)+1)
        self.adjustment_z = (((asaz-128)*0.5/128)+1)

    def mag_calibration(self, s: int):
        """
        Calculate the magnetometer bias, scale, misalignment, geomagnetic field strength with four element calibration
        Using least square method to solve the error

        :param int s: time for calibration
        :returns: 
            - mag_scale (ndarray) - 3-axis magnetometer scale (soft-iron offset)
            - mag_bias (ndarray) - 3-axis magnetometer bias (hard-iron offset)
            - mag_misalignment (ndarray) - 3-axis magnetometer misalignment (soft-iron offset)
            - mag_strength (float) - geomagnetic field strength

        .. Reference
        .. [1] `four element calibration <https://www.nxp.com/docs/en/application-note/AN5019.pdf>`
        .. [2] `ten element calibration <https://github.com/nliaudat/magnetometer_calibration/blob/main/calibrate.py>`
        """
        if s > 0 and self.calibration == True:
            input("Please move the IMU in slow motion in all possible directions, the calibration process takes {}s".format(s))
            calibration = []
            target = []
            for i in range(s*self.hz):
                mx, my, mz = self.get_mag()
                calibration.append([mx, my, mz, 1])
                target.append([mx**2+my**2+mz**2])
                time.sleep(1/self.hz)
            calibration = np.array(calibration)
            target = np.array(target)

            error_matrix = np.linalg.inv(calibration.T @ calibration) @ calibration.T @ target

            # calculate bias
            x_bias = 0.5*error_matrix[0][0]
            y_bias = 0.5*error_matrix[1][0]
            z_bias = 0.5*error_matrix[2][0]
            self.mag_bias = np.array([[x_bias],[y_bias],[z_bias]])

            # calculate scale
            self.mag_scale = np.array([[1],[1],[1]])

            # calculate misalignment
            self.mag_misalignment = np.array([[0],[0],[0],[0],[0],[0]])

            # calculate strength
            strength = (error_matrix[3][0]+x_bias**2+y_bias**2+z_bias**2)**0.5
            self.mag_strength = np.array([[strength]])
        return self.mag_scale, self.mag_bias, self.mag_misalignment, self.mag_strength

    def get_mag(self):
        """
        AK8963 magnetometer data in Earth's reference (µT)

        ENU: \n
        mx is positive when the right hand side pointing to north \n
        my is positive when the front side pointing to north \n
        mz is positive when the upper side pointing to north \n

        NED: \n
        mx is positive when the front side pointing to north \n
        my is positive when the right hand side pointing to north \n
        mz is positive when the under side pointing to north \n
    
        :returns: 
            - mx (float) - x-axis magnetometer data in µT
            - my (float) - y-axis magnetometer data in µT
            - mz (float) - z-axis magnetometer data in µT
        """
        try:
            mx = self.read_raw_data(HXH, HXL)*self.mag_fs
            my = self.read_raw_data(HYH, HYL)*self.mag_fs
            mz = self.read_raw_data(HZH, HZL)*self.mag_fs
            # print("raw mx my mz: ", mx, my, mz)
        except:
            raise ConnectionError("I2C Connection Failure")

        # sensitivity adjustment
        mx = mx*self.adjustment_x       
        my = my*self.adjustment_y
        mz = mz*self.adjustment_z

        ST2_value = self.bus.read_byte_data(self.address, ST2)
        bit_3 = ST2_value & int("00001000", 2)
        bit_4 = ST2_value & int("00010000", 2)

        # if bit_3 == 0:
        #     print("Ready in measurement data register or ST2 register")
        # elif bit_3 == 8:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        #     print("Magnetic sensor overflow occurred")
        # else:
        #     raise ValueError("AK8963 status 2 register bit 3 error")

        # if bit_4 == 0:
        #     print("Magnetic sensor in 14-bit coding")
        # elif bit_4 == 16:
        #     print("Magnetic sensor in 16-bit coding")
        # else:
        #     raise ValueError("AK8963 status 2 register bit 4 error")

        if self.nav_frame == "ENU":
            mx, my, mz = NED2ENU(mx, my, mz)

        # magnetometer model: calibrated measurement = (matrix)*(raw measurement - bias)
        x_scale = self.mag_scale[0][0]
        y_scale = self.mag_scale[1][0]
        z_scale = self.mag_scale[2][0]
        x_bias = self.mag_bias[0][0]
        y_bias = self.mag_bias[1][0]
        z_bias = self.mag_bias[2][0]
        xy_mis = self.mag_misalignment[0][0]
        xz_mis = self.mag_misalignment[1][0]
        yx_mis = self.mag_misalignment[2][0]
        yz_mis = self.mag_misalignment[3][0]
        zx_mis = self.mag_misalignment[4][0]
        zy_mis = self.mag_misalignment[5][0]
        if self.calibration == False:
            mx = (x_scale * mx + xy_mis * my + xz_mis * mz) - x_bias
            my = (yx_mis * mx + y_scale * my + yz_mis * mz) - y_bias
            mz = (zx_mis * mx + zy_mis * my + z_scale * mz) - z_bias
        return mx, my, mz

    def set_mode(self, mode, bit):
        """
        AK8963 CNTL1 register configuration

        :param bool mode: operation mode setting
        :param int bit: output bit setting 
        """
        # power down mode
        if mode == "power down": # 00000000
            value = 0x00
        elif mode == "single measure": # 00000001
            value = 0x01
        elif mode == "continuous measure 1": # 00000010
            value = 0x02 # 8Hz
        elif mode == "continuous measure 2": # 00000110
            value = 0x06 # 100Hz
        elif mode == "external trigger measurement": # 00000100
            value = 0x04
        elif mode == "self test": # 00001000
            value = 0x08
        elif mode == "fuse rom access": # 00001111 
            value = 0x0F
        else:
            raise ValueError("Prohibit mode coding")
        
        if bit == 14:
            value += 0
        elif bit == 16:
            value += 16
        else:
            raise ValueError("Wrong bit coding")

        self.mag_fs = 4912*2/(2**bit)
        print("Set AK8963 to {} mode".format(mode))
        self.bus.write_byte_data(self.address, CNTL, value)
        
    def read_raw_data(self, high_register, low_register):
        """
        Access the high and low registers of the magnetometer to calculate their values into int16 format

        :param int high_register: high registers of the magnetometer
        :param int low_register: low registers of the magnetometer

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