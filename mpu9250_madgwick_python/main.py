#!/usr/bin/python3
from mpu9250 import MPU9250
from madgwick import Madgwick
from orientation import quat2eul, eul2quat
from visualization import imu_viewer
import time
import numpy as np
import math

# ENU (right hand rule):
# X (Pitch): clockwise - / anticlockwise +
# Y (Roll): clockwise - / anticlockwise +
# Z (Yaw): clockwise - / anticlockwise +

# NED:
# X (Roll): clockwise + / anticlockwise -
# Y (Pitch): clockwise + / anticlockwise -
# Z (Yaw): clockwise + / anticlockwise -

def imu_thread(self):
    axis = 9
    hz = 100
    nav_frame = "NED" # ENU/NED
    imu = MPU9250(nav_frame, axis, hz)
    try:
        while True:
            next = last + interval
            time.sleep(abs(next - time.time()))
            ax, ay, az = imu.get_accel()
            gx, gy, gz = imu.get_gyro()
            mx, my, mz = imu.get_mag()
            roll, pitch, yaw = imu.get_euler(axis=axis)
            w, x, y, z = imu.get_quaternion(axis=axis)
            temp = imu.get_temp()
            ax, ay, az = round(ax,2), round(ay,2), round(az,2)
            gx, gy, gz = round(gx,2), round(gy,2), round(gz,2)
            mx, my, mz = round(mx,2), round(my,2), round(mz,2)
            roll, pitch, yaw = round(roll,2), round(pitch,2), round(yaw,2)
            acc = np.array([[ax],[ay],[az]])
            gyr = np.array([[gx],[gy],[gz]])
            mag = np.array([[mx],[my],[mz]])
            euler = np.array([[roll],[pitch],[yaw]])
            quaternion = np.array([[w],[x],[y],[z]])
            temp = round(temp, 2)
            pv.run(euler, quaternion, body_frame, nav_frame)
            print("")
            print("nav_frame: ", nav_frame)
            print("ax ay az: ", ax, ay, az)
            print("gx gy gz: ", gx, gy, gz)
            print("mx my mz: ", mx, my, mz)
            print("roll pitch yaw: ", roll, pitch, yaw)
            last = time.time()
    except KeyboardInterrupt:
        print('interrupted!')

if __name__ == '__main__':
    imu = threading.Thread(target=imu_thread)
    imu.start()
    viewer.start()










# if __name__ == "__main__":
#     nav_frame = "NED" # ENU/NED
#     axis = 9
#     hz = 100
#     interval = 1/hz
#     last = time.time()

#     imu = MPU9250(nav_frame=nav_frame)
#     filter = Madgwick(axis, marg_gain=5)
#     pv = imu_viewer(1080, 720, hz)
#     # imu.mpu6500.gyro_calibration(500, 100)
#     ax, ay, az = imu.get_accel()
#     mx, my, mz = imu.get_mag()
#     roll, pitch, yaw = imu.get_rpy(axis=axis)
#     if nav_frame=="NED":
#         w, x, y, z = eul2quat(roll, pitch, yaw, seq="zyx")
#     elif nav_frame=="ENU":
#         w, x, y, z = eul2quat(roll, pitch, yaw, seq="zxy")
#     filter.init_quat(w, x, y, z)

#     try:
#         while True:
#             next = last + interval
#             # print("dt: ", abs(next - time.time()))
#             # print("Hz: ", 1/abs(next - time.time()))
#             time.sleep(abs(next - time.time()))
#             ax, ay, az = imu.get_accel()
#             gx, gy, gz = imu.get_gyro()
#             mx, my, mz = imu.get_mag()
#             roll, pitch, yaw = imu.get_rpy(axis=axis)
#             temp = imu.get_temp()
#             ax, ay, az = round(ax,2), round(ay,2), round(az,2)
#             gx, gy, gz = round(gx,2), round(gy,2), round(gz,2)
#             mx, my, mz = round(mx,2), round(my,2), round(mz,2)
#             roll, pitch, yaw = round(roll,2), round(pitch,2), round(yaw,2)
#             if nav_frame=="NED":
#                 w, x, y, z = eul2quat(roll, pitch, yaw, seq="zyx")
#                 body_frame = "FRD"
#             elif nav_frame=="ENU":
#                 w, x, y, z = eul2quat(roll, pitch, yaw, seq="zxy")
#                 body_frame = "RFU"
#             pv.run(np.array([[roll],[pitch],[yaw]]), np.array([[w],[x],[y],[z]]), body_frame, nav_frame)
#             temp = round(temp, 2)
#             print("")
#             print("nav_frame: ", nav_frame)
#             print("ax ay az: ", ax, ay, az)
#             print("gx gy gz: ", gx, gy, gz)
#             print("mx my mz: ", mx, my, mz)
#             acc = np.array([[ax],[ay],[az]])
#             gyr = np.array([[gx],[gy],[gz]])
#             mag = np.array([[mx],[my],[mz]])
#             print("roll pitch yaw: ", roll, pitch, yaw)
#             w, x, y, z = filter.run(acc, gyr, mag, hz)
#             if nav_frame=="NED":
#                 roll, pitch, yaw = quat2eul(w, x, y, z, seq="zyx")
#             elif nav_frame=="ENU":
#                 roll, pitch, yaw = quat2eul(w, x, y, z, seq="zxy")
#             roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
#             roll, pitch, yaw = round(roll,2), round(pitch,2), round(yaw,2)
#             # print("madgwick roll pitch yaw: ", roll, pitch, yaw)
#             last = time.time()
#     except KeyboardInterrupt:
#         print('interrupted!')
