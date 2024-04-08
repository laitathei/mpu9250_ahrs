import math
import numpy as np
from utils.orientation import quat2dcm, quat_multi, quat2eul

class Mahony():
    """
    Mahony is one of the AHRS filter applied with complementary filter [1]_

    :param int axis: axis data for fusion
    :param float kp: proportional gain
    :param float ki: integral gain
    :param str nav_frame: navigation frame

    .. Reference
    .. [1] 'Mahony <https://ahrs.readthedocs.io/en/latest/filters/mahony.html#ahrs.filters.mahony.Mahony.updateIMU>'
    """
    def __init__(self, axis, kp=0.1, ki=0.1, nav_frame="NED"):
        # Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
        # Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

        # algorithm parameter
        self.axis = axis # 6 or 9
        self.ki = ki # reduce the steady-state error
        self.kp = kp # increasing trusts more accelerometers and magnetometers, decreasing trusts more gyroscopes
        self.gyro_bias = np.zeros((3,1))
        self.nav_frame = nav_frame # ENU or NED
        if self.nav_frame != "ENU" and self.nav_frame != "NED":
            raise ValueError("Navigation frame must be ENU or NED")

        if self.axis != 6 and self.axis != 9:
            raise ValueError("Axis must be 6 or 9")

        print("Mahony filter in use")

    def init_quat(self, w, x, y, z):
        """
        Mahony filter initial attitude

        :param float w: Quaternion magnitude
        :param float x: Quaternion X axis
        :param float y: Quaternion Y axis
        :param float z: Quaternion Z axis
        """
        self.est_quat = np.array([[w],[x],[y],[z]])

    def run(self, acc, gyr, mag, hz):
        """
        Iteration of Mahony filter

        :param ndarray acc: accelerometer data
        :param ndarray gyr: gyroscope data
        :param ndarray mag: magnetometer data
        :param int hz: IMU frequency
        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        if acc.shape != (3,1):
            raise ValueError("acc shape must be (3,1)")
        elif gyr.shape != (3,1):
            raise ValueError("gyr shape must be (3,1)")
        elif mag.shape != (3,1):
            raise ValueError("mag shape must be (3,1)")

        self.imu_hz = hz
        self.imu_dt = 1/self.imu_hz
        self.acc = acc
        self.gyr = gyr
        self.mag = mag
        if self.axis == 6: # 6 axis fusion (gyroscope and accelerometer)
            w, x, y, z = self.gyro_acc_fusion()
        elif self.axis == 9: # 9 axis fusion (gyroscope, accelerometer and magnetometer)
            w, x, y, z = self.gyro_acc_mag_fusion()
        return w, x, y, z

    def gyro_acc_fusion(self):
        """
        Mahony filter 6 axis data fusion

        ENU: \n
        Gravity is defined as negative when pointing upwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n

        NED: \n
        Gravity is defined as negative when pointing downwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        a_norm = np.linalg.norm(acc)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        DCM = quat2dcm(qw, qx, qy, qz) # DCM from body frame to navigation frame
        if a_norm > 0:
            # accelerometer part
            a = acc/a_norm # normalize acceleration vector
            a = np.squeeze(a.T)
            if self.nav_frame == "ENU":
                v_g = DCM.T @ np.array([0.0, 0.0, 1.0]) # convert expected gravity vector from navigation frame to body frame
            elif self.nav_frame == "NED":
                v_g = DCM.T @ np.array([0.0, 0.0, -1.0]) # convert expected gravity vector from navigation frame to body frame
            acc_error = np.cross(a, v_g) # error of accelerometer (gravity vector in body frame cross prodcut with accelerometer vector)

            # Mahony algorithm
            total_error = np.expand_dims(acc_error, axis=1)
            self.gyro_bias = self.gyro_bias + self.ki * total_error * self.imu_dt # estimate current gyroscope bias by adding past changes
            gyr = gyr + self.gyro_bias + self.kp * total_error # gyroscope correction
            gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z

    def gyro_acc_mag_fusion(self):
        """
        Mahony filter 9 axis data fusion

        ENU: \n
        Gravity is defined as negative when pointing upwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n
        Magnetometer data in Earth's reference (µT) \n

        NED: \n
        Gravity is defined as negative when pointing downwards \n
        Accelerometer in Earth's reference (m/s^2) \n
        Gyroscope in right hand coordinates (rad/s) \n
        Magnetometer data in Earth's reference (µT) \n

        :returns: 
            - w (float) - Quaternion magnitude
            - x (float) - Quaternion X axis
            - y (float) - Quaternion Y axis
            - z (float) - Quaternion Z axis
        """
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        mag = np.copy(self.mag)
        a_norm = np.linalg.norm(acc)
        m_norm = np.linalg.norm(mag)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        DCM = quat2dcm(qw, qx, qy, qz) # DCM from body frame to navigation frame
        if a_norm > 0 and m_norm > 0:
            # accelerometer part
            a = acc/a_norm # normalize acceleration vector
            a = np.squeeze(a.T)
            if self.nav_frame == "ENU":
                v_g = DCM.T @ np.array([0.0, 0.0, 1.0]) # convert expected gravity vector from navigation frame to body frame
            elif self.nav_frame == "NED":
                v_g = DCM.T @ np.array([0.0, 0.0, -1.0]) # convert expected gravity vector from navigation frame to body frame
            acc_error = np.cross(a, v_g) # error of accelerometer (gravity vector in body frame cross prodcut with accelerometer vector)

            # magnetometer part
            m = mag/m_norm # normalize magnetometer vector
            m = np.squeeze(m.T)
            h = DCM.T @ m # convert magnetic field from body frame to navigation frame
            if self.nav_frame == "ENU":
                # In ENU frame, Y axis in body frame align to north, X axis will 0
                b = np.array([[0], [np.linalg.norm([h[0], h[1]])], [h[2]]])
            elif self.nav_frame == "NED":
                # In NED frame, X axis in body frame align to north, Y axis will 0
                b = np.array([[np.linalg.norm([h[0], h[1]])], [0], [h[2]]])
            v_m = DCM.T @ b # convert the aligned magnetometer vector from navigation frame to body frame
            v_m = np.squeeze(v_m.T)
            mag_error = np.cross(m, v_m) # error of magnetometer (aligned magnetometer vector in body frame cross prodcut with origin magnetometer vector)

            # Mahony algorithm
            total_error = np.expand_dims(acc_error, axis=1) + np.expand_dims(mag_error, axis=1)
            self.gyro_bias = self.gyro_bias + self.ki * total_error * self.imu_dt # estimate current gyroscope bias by adding past changes
            gyr = gyr + self.gyro_bias + self.kp * total_error # gyroscope correction
            gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z