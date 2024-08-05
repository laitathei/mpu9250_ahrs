#!/usr/bin/python3
from mpu9250_driver.mpu9250 import MPU9250
from ahrs import madgwick, mahony, ekf
import time
import numpy as np
import math

if __name__ == '__main__':
    """
    Display all the information about MPU9250
    """
    nav_frame = "NED" # ENU/NED
    axis = 9
    hz = 100
    interval = 1/hz
    calibration = False
    # ahrs = madgwick.Madgwick(axis, 1, nav_frame)
    # ahrs = mahony.Mahony(axis, 0.1, 0, nav_frame)
    # ahrs = ekf.EKF(axis, [0.3**2, 0.5**2, 0.8**2], nav_frame)
    imu = MPU9250(nav_frame, axis, hz, calibration)
    imu.initialization()
    imu.start_thread(ahrs=None)
    last = time.time()
    try:
        while True:
            next = last + interval
            time.sleep(abs(next - time.time()))
            print("")
            print("temp: ", round(imu.temp,5))
            print("nav_frame: ", nav_frame)
            print("ax ay az: ", round(imu.ax,5), round(imu.ay,5), round(imu.az,5))
            print("gx gy gz: ", round(imu.gx,5), round(imu.gy,5), round(imu.gz,5))
            print("mx my mz: ", round(imu.mx,5), round(imu.my,5), round(imu.mz,5))
            print("roll pitch yaw: ", round(imu.roll,5), round(imu.pitch,5), round(imu.yaw,5))
            print("w x y z: ", round(imu.w,5), round(imu.x,5), round(imu.y,5), round(imu.z,5))
            last = time.time()
    except KeyboardInterrupt:
        print('interrupted!')