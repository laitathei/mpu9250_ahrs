#!/usr/bin/python
import smbus
import math
import numpy as np
from transformation import NED2ENU, ENU2NED

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
    def __init__(self, bus, address, nav_frame="NED"):
        # original navigation frame of AK8963 is NED
        self.bus = bus
        self.address = address
        self.mag_offset = np.zeros((3,1))
        self.nav_frame = nav_frame

    def who_am_i(self):
        value = hex(self.bus.read_byte_data(self.address, WIA))
        print("The register value is {}".format(value))
        if value == "0x48":
            print("It is AK8963 default value")
        else:
            print("It is not AK8963 default value")
            raise RuntimeError("AK8963 not found")

    def config_AK8963(self, bit):
        self.set_mode("fuse rom access", 16)
        self.get_adjust_mag()
        self.set_mode("power down", 16)
        self.set_mode("continuous measure 2", 16)
        self.get_status()

    def get_status(self):
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
        print("Read sensitivity adjustment value")
        self.asax = self.bus.read_byte_data(self.address, ASAX)
        self.asay = self.bus.read_byte_data(self.address, ASAY)
        self.asaz = self.bus.read_byte_data(self.address, ASAZ)

    def get_mag(self):
        try:
            mx = self.read_raw_data(HXH, HXL)*self.mag_scale
            my = self.read_raw_data(HYH, HYL)*self.mag_scale
            mz = self.read_raw_data(HZH, HZL)*self.mag_scale
            # print("raw mx my mz: ", mx, my, mz)
        except:
            raise ConnectionError("I2C Connection Failure")

        # sensitivity adjustment
        mx = mx*(((self.asax-128)*0.5/128)+1)        
        my = my*(((self.asay-128)*0.5/128)+1) 
        mz = mz*(((self.asaz-128)*0.5/128)+1) 

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
        return mx, my, mz

    def set_mode(self, mode, bit):
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

        self.mag_scale = 4912*2/(2**bit)
        print("Set AK8963 to {} mode".format(mode))
        self.bus.write_byte_data(self.address, CNTL, value)
        
    def read_raw_data(self, high_register, low_register):
        high = self.bus.read_byte_data(self.address, high_register)
        low = self.bus.read_byte_data(self.address, low_register)

        # Megre higher bytes and lower bytes data
        value = (high << 8) + low

        # Calculate the unsigned int16 range to signed int16 range
        if(value > 32767):
            value = value - 65536

        return value
