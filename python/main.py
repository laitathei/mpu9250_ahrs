#!/usr/bin/python3
from mpu9250_driver.mpu9250 import MPU9250
from utils.orientation import quat2eul, eul2quat
from ahrs import madgwick, mahony
import time
import numpy as np
import math

if __name__ == '__main__':
    """
    Display all the information about MPU9250
    """
    nav_frame = "NED" # ENU/NED
    axis = 9
    gain = 1
    hz = 100
    interval = 1/hz
    calibration = False
    ahrs = Madgwick(axis, gain)
    imu = MPU9250(nav_frame, axis, hz, calibration)
    imu.initialization()
    imu.start_thread(ahrs)
    last = time.time()
    try:
        while True:
            next = last + interval
            time.sleep(abs(next - time.time()))
            print("")
            print("nav_frame: ", nav_frame)
            print("ax ay az: ", imu.ax, imu.ay, imu.az)
            print("gx gy gz: ", imu.gx, imu.gy, imu.gz)
            print("mx my mz: ", imu.mx, imu.my, imu.mz)
            print("roll pitch yaw: ", imu.roll, imu.pitch, imu.yaw)
            print("w x y z: ", imu.w, imu.x, imu.y, imu.z)
            last = time.time()
    except KeyboardInterrupt:
        print('interrupted!')