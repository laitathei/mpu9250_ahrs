import math
import numpy as np

from orientation import quat_multi, quat_conjugate, quat2eul
from transformation import acc2quat, accmag2quat

class Madgwick():
    def __init__(self, axis, imu_gain=0.033, marg_gain=0.041, nav_frame="NED"):
        """
        Madgwick is one of the orientation Filter applied gradient descent technique
        
        .. Reference
        .. [1] https://ahrs.readthedocs.io/en/latest/filters/madgwick.html#orientation-from-angular-rate
        """
        # Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
        # Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

        # algorithm parameter
        self.axis = axis # 6 or 9
        self.imu_gain = imu_gain
        self.marg_gain = marg_gain # magnetic, angular rate and gravity

        self.nav_frame = nav_frame # ENU or NED
        if self.nav_frame != "ENU" and self.nav_frame != "NED":
            raise ValueError("Navigation frame must be ENU or NED")

        # imu parameter
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.gx = 0
        self.gy = 0
        self.gz = 0
        self.mx = 0
        self.my = 0
        self.mz = 0
        if self.axis != 6 and self.axis != 9:
            raise ValueError("Axis must be 6 or 9")
        
    def init_quat(self, w, x, y, z):
        self.est_quat = np.array([[w],[x],[y],[z]])

    def run(self, acc, gyr, mag, hz):
        self.imu_hz = hz
        self.imu_dt = 1/self.imu_hz
        self.acc = acc
        self.gyr = gyr
        self.mag = mag
        if self.axis == 6: # 6 axis fusion (gyroscope and accelerometer)
            self.gain = self.imu_gain # Beta
            w, x, y, z = self.gyro_acc_fusion()
        elif self.axis == 9: # 9 axis fusion (gyroscope, accelerometer and magnetometer)
            self.gain = self.marg_gain # Beta
            w, x, y, z = self.gyro_acc_mag_fusion()
        return w, x, y, z

    def gyro_acc_fusion(self):
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        a_norm = np.linalg.norm(acc)
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        if a_norm > 0:
            a = np.copy(acc)/a_norm # normalize acceleration vector
            q_norm = np.linalg.norm(self.est_quat)
            self.est_quat = self.est_quat/q_norm # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
            ax, ay, az = a[0][0], a[1][0], a[2][0]
            f = np.array([[2.0*(qx*qz - qw*qy) - ax],     # calculate objective function
                          [2.0*(qw*qx + qy*qz) - ay],
                          [2.0*(0.5-qx**2-qy**2) - az]])
            J = np.array([[-2.0*qy, 2.0*qz, -2.0*qw, 2.0*qx],   # calculate Jacobian form
                          [2.0*qx,  2.0*qw, 2.0*qz,  2.0*qy],
                          [0.0,    -4.0*qx, -4.0*qy, 0.0]])
            gradient = J.T@f
            gradient_norm = np.linalg.norm(gradient)
            gradient = gradient/gradient_norm # normalize gradient
            quat_change = quat_change - self.gain * gradient # compute new quaternion after gradient descent
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        q_norm = np.linalg.norm(self.est_quat)
        self.est_quat = self.est_quat/q_norm # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z

    def gyro_acc_mag_fusion(self):
        acc = np.copy(self.acc)
        gyr = np.copy(self.gyr)
        mag = np.copy(self.mag)
        gx, gy, gz = gyr[0][0], gyr[1][0], gyr[2][0]
        a_norm = np.linalg.norm(acc)
        m_norm = np.linalg.norm(mag)
        origin_q = np.array([[0],[gx],[gy],[gz]]) # the current reading of gyroscope in quaternion form
        quat_diff = quat_multi(self.est_quat, origin_q)
        quat_change = 0.5 * quat_diff # compute the rate of change of quaternion
        if a_norm > 0 and m_norm > 0:
            a = np.copy(acc)/a_norm # normalize acceleration vector
            m = np.copy(mag)/m_norm # normalize magnetometer vector
            mx, my, mz = m[0][0], m[1][0], m[2][0]
            m = np.array([[0],[mx],[my],[mz]])
            h = quat_multi(self.est_quat, quat_multi(m, quat_conjugate(self.est_quat)))               # (eq. 45)
            if self.nav_frame == "ENU":
                # Madgwick assumes the East component of the magnetic field is negligible
                # In ENU frame, Y axis in body frame align to north, X axis will 0
                b = np.array([[0], [np.linalg.norm([h[1][0], h[2][0]])], [h[3][0]]])
            elif self.nav_frame == "NED":
                # Madgwick assumes the East component of the magnetic field is negligible
                # In NED frame, X axis in body frame align to north, Y axis will 0
                b = np.array([[np.linalg.norm([h[1][0], h[2][0]])], [0], [h[3][0]]])
            q_norm = np.linalg.norm(self.est_quat) # normalize quaternion
            self.est_quat = self.est_quat/q_norm # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
            bx, by, bz = b[0][0], b[1][0], b[2][0]
            ax, ay, az = a[0][0], a[1][0], a[2][0]
            f = np.array([[2.0*(qx*qz - qw*qy) - ax],     # calculate objective function
                          [2.0*(qw*qx + qy*qz) - ay],
                          [2.0*(0.5-qx**2-qy**2) - az],
                          [2.0*bx*(0.5 - qy**2 - qz**2) + 2.0*by*(qw*qz + qx*qy)         + 2.0*bz*(qx*qz - qw*qy)       - mx],
                          [2.0*bx*(qx*qy - qw*qz)       + 2.0*by*(0.5 - qx**2 - qz**2)   + 2.0*bz*(qw*qx + qy*qz)       - my],
                          [2.0*bx*(qw*qy + qx*qz)       + 2.0*by*(qy*qz - qw*qx)         + 2.0*bz*(0.5 - qx**2 - qy**2) - mz]])
            J = np.array([[-2.0*qy,                2.0*qz,                          -2.0*qw,                          2.0*qx],       # calculate Jacobian form
                            [ 2.0*qx,                2.0*qw,                          2.0*qz,                           2.0*qy],
                            [ 0.0,                   -4.0*qx,                         -4.0*qy,                          0.0],
                            [ 2.0*by*qz-2.0*bz*qy,   2.0*by*qy+2.0*bz*qz,             -4.0*bx*qy+2.0*by*qx-2.0*bz*qw,   -4.0*bx*qz+2.0*by*qw+2.0*bz*qx],
                            [-2.0*bx*qz+2.0*bz*qx,   2.0*bx*qy-4.0*by*qx+2.0*bz*qw,   2.0*bx*qx+2.0*bz*qz,              -2.0*bx*qw-4.0*by*qz+2.0*bz*qy],
                            [ 2.0*bx*qy-2.0*by*qx,   2.0*bx*qz-2.0*bx*qw-4.0*bz*qx,   2.0*bx*qw+2.0*by*qz-4.0*bz*qy,    2.0*bx*qx+2.0*by*qy]])
            gradient = J.T@f
            gradient_norm = np.linalg.norm(gradient)
            gradient = gradient/gradient_norm # normalize gradient
            quat_change = quat_change - self.gain * gradient # compute new quaternion after gradient descent
        self.est_quat = self.est_quat + quat_change * self.imu_dt # update the quaternion
        q_norm = np.linalg.norm(self.est_quat)
        self.est_quat = self.est_quat/q_norm # normalize quaternion vector
        w, x, y, z = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return w, x, y, z

