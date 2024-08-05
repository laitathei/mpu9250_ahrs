#!/usr/bin/python3
import numpy as np
import scipy as sp
import math
from utils.orientation import quat2dcm, quat2eul
from utils.transformation import skew_symmetric

class EKF():
    """
    Extended Kalman filter is one of the AHRS filter dealing with sensor gaussian noise [1]_

    :param int axis: axis data for fusion
    :param list noise: gyroscope, accelerometer, magnetometer gaussian noise
    :param str nav_frame: navigation frame

    .. Reference
    .. [1] 'EKF <https://ahrs.readthedocs.io/en/latest/filters/ekf.html>'
    """
    def __init__(self, axis, noise=[0.3**2, 0.5**2, 0.8**2], nav_frame="NED"):
        # Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
        # Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

        # algorithm parameter
        self.axis = axis # 6 or 9
        self.nav_frame = nav_frame # ENU or NED
        self.gyro_noise = noise[0] # increase noise means gyroscope measurement not accurate, decrease noise means gyroscope measurement accurate
        self.accel_noise = noise[1] # increae noise means accelerometer measurement not accurate, decrease noise means accelerometer measurement accurate
        self.mag_noise = noise[2] # increase noise means magnetometer measurement not accurate, decrease noise means magnetometer measurement accurate
        if self.axis == 6:
            v = np.array([[self.accel_noise]]) # measurement noise vector
        elif self.axis == 9:
            v = np.array([[self.accel_noise], [self.mag_noise]]) # measurement noise vector
        self.R = np.diag(np.repeat(v, 3)) # measurement noise covariance matrix
        self.I = np.identity(4) # identity matrix
        self.P = np.identity(4) # initial state covariance

        # Hong Kong geomagnetic field parameter
        # http://www.geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html
        # self.declination = -3.404 # declination angle (degree)
        # self.inclination = 33.739 # inclination angle (degree)
        # self.north_intensity = 37794 # north intensity (nT)
        # self.east_intensity = -2248 # east intensity (nT)
        # self.horizontal_intensity = 37861 # horizontal intensity (nT)
        # self.vertical_intensity = 25287 # vertical intensity (nT)
        # self.total_intensity = 45529 # total intensity (nT)

        # Waterloo, Canada geomagnetic field parameter
        self.declination = -9.434 # declination angle (degree)
        self.inclination = 69.037 # inclination angle (degree)
        self.north_intensity = 18780 # north intensity (nT)
        self.east_intensity = -3120 # east intensity (nT)
        self.horizontal_intensity = 19037 # horizontal intensity (nT)
        self.vertical_intensity = 49688 # vertical intensity (nT)
        self.total_intensity = 53210 # total intensity (nT)

        self.a_ref = np.array([[0],[0],[1]]) # Gravitational Reference Vector (due to my accelerometer definition)
        if self.nav_frame == "ENU":
            self.m_ref = np.array([[0],[math.cos(math.radians(self.inclination))],[-math.sin(math.radians(self.inclination))]]) # Magnetic Reference Vector
        elif self.nav_frame == "NED":
            self.m_ref = np.array([[math.cos(math.radians(self.inclination))],[0],[math.sin(math.radians(self.inclination))]]) # Magnetic Reference Vector
        self.m_ref = self.m_ref/np.linalg.norm(self.m_ref)

        if self.nav_frame != "ENU" and self.nav_frame != "NED":
            raise ValueError("Navigation frame must be ENU or NED")

        if self.axis != 6 and self.axis != 9:
            raise ValueError("Axis must be 6 or 9")
            
        print("Extended Kalman filter in use")

    def init_quat(self, w, x, y, z):
        """
        Extended Kalman filter initial attitude

        :param float w: Quaternion magnitude
        :param float x: Quaternion X axis
        :param float y: Quaternion Y axis
        :param float z: Quaternion Z axis
        """
        self.est_quat = np.array([[w],[x],[y],[z]])
        
    def run(self, acc, gyr, mag, hz):
        """
        Iteration of Extended Kalman filter

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
        if self.axis == 6:
            w, x, y, z = self.gyro_acc_fusion() # 6 axis fusion (gyroscope, accelerometer)
        elif self.axis == 9:
            w, x, y, z = self.gyro_acc_mag_fusion() # 9 axis fusion (gyroscope, accelerometer and magnetometer)
        return w, x, y, z

    def gyro_acc_fusion(self):
        """
        Extended Kalman filter 6 axis data fusion

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
        mag = np.copy(self.mag)
        a_norm = np.linalg.norm(acc)
        m_norm = np.linalg.norm(mag)
        qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        if a_norm > 0:
            a = acc/a_norm # normalize acceleration vector
            # ----- Prediction -----
            f = self.f(gyr) # Predicted State
            F = self.F(gyr) # Discreted State Transition Matrix
            W = 0.5*self.imu_dt*np.array([[-qx,-qy,-qz], # Jacobian W = df/dω
                                          [qw,-qz,qy],
                                          [qz,qw,-qx],
                                          [-qy,qx,qw]])
            Q = self.gyro_noise * W @ W.T # Process Noise Covariance Matrix
            P = F @ self.P @ F.T + Q # Predicted Covariance Matrix
            # ----- Correction -----
            z = a
            y   = self.h(f) # Expected Measurement Function
            v   = z - y # Innovation (Measurement Residual)
            H   = self.H(f) # Discreted Measurement Matrix
            S   = H @ P @ H.T + self.R # Measurement Prediction Covariance
            # Replace np.linalg.inv by sp.linalg.inv for accelerating matrix calculation
            K   = P @ H.T @ sp.linalg.inv(S) # Kalman Gain 
            self.P = (self.I - K @ H) @ P # Updated Covariance Matrix
            self.est_quat = f + K @ v # Corrected State
            self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return qw, qx, qy, qz

    def gyro_acc_mag_fusion(self):
        """
        Extended Kalman filter 9 axis data fusion

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
        mag = np.copy(self.mag)
        a_norm = np.linalg.norm(acc)
        m_norm = np.linalg.norm(mag)
        qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        if a_norm > 0 and m_norm > 0:
            a = acc/a_norm # normalize acceleration vector
            m = mag/m_norm # normalize magnetometer vector
            # ----- Prediction -----
            f = self.f(gyr) # Predicted State
            F = self.F(gyr) # Discreted State Transition Matrix
            W = 0.5*self.imu_dt*np.array([[-qx,-qy,-qz], # Jacobian W = df/dω
                                          [qw,-qz,qy],
                                          [qz,qw,-qx],
                                          [-qy,qx,qw]])
            Q = self.gyro_noise * W @ W.T # Process Noise Covariance Matrix
            P = F @ self.P @ F.T + Q # Predicted Covariance Matrix
            # ----- Correction -----
            z = np.vstack((a, m))
            y   = self.h(f) # Expected Measurement Function
            v   = z - y # Innovation (Measurement Residual)
            H   = self.H(f) # Discreted Measurement Matrix
            S   = H @ P @ H.T + self.R # Measurement Prediction Covariance
            # Replace np.linalg.inv by sp.linalg.inv for accelerating matrix calculation
            K   = P @ H.T @ sp.linalg.inv(S) # Kalman Gain 
            self.P = (self.I - K @ H) @ P # Updated Covariance Matrix
            self.est_quat = f + K @ v # Corrected State
            self.est_quat = self.est_quat/np.linalg.norm(self.est_quat) # normalize quaternion vector
            qw, qx, qy, qz = self.est_quat[0][0], self.est_quat[1][0], self.est_quat[2][0], self.est_quat[3][0]
        return qw, qx, qy, qz

    def quat_differential(self, gyro):
        """
        Calculate differential equation for quaternion

        :param ndarray gyro: gyroscope data in rad/s
        :returns: 
            - matrix (ndarray) - differential equation for quaternion
        """
        #          [0 , -wx, -wy, -wz]
        # matrix = [wx,  0 ,  wz, -wy]
        #          [wy, -wz,  0 ,  wx]
        #          [wz,  wy, -wx,  0 ]
        wx, wy, wz = gyro[0][0], gyro[1][0], gyro[2][0]
        matrix = np.zeros((4,4))
        skew = skew_symmetric(wx, wy, wz)
        vector = np.array([0,wx,wy,wz])
        matrix[:,0] = vector
        matrix[0,:] = -vector
        matrix[1:,1:] = skew.T
        return matrix

    def f(self, gyro):
        """
        Process model - linearized dynamic model function (predicted state)

        :param ndarray gyro: gyroscope data in rad/s
        :returns: 
            - f (ndarray) - predicted quaternion for next time step
        """
        #     [qw - wx*qx*dt/2 - wy*qy*dt/2 - wz*qz*dt/2]
        # f = [qx + wx*qw*dt/2 - wy*qz*dt/2 + wz*qy*dt/2]
        #     [qy + wx*qz*dt/2 + wy*qw*dt/2 - wz*qx*dt/2]
        #     [qz - wx*qy*dt/2 + wy*qx*dt/2 + wz*qw*dt/2]
        matrix = self.I + (0.5*self.imu_dt*self.quat_differential(gyro))
        f = matrix @ self.est_quat
        return f

    def F(self, gyro):
        """
        Process model - Jacobian of linearized dynamic model function (predicted state)

        :param ndarray gyro: gyroscope data in rad/s
        :returns: 
            - F (ndarray) - Discrete time state transition matrix
        """
        #     [1      , -wx*dt/2, -wy*dt/2, -wz*dt/2]
        # F = [wx*dt/2,     1   , wz*dt/2,  -wy*dt/2]
        #     [wy*dt/2, -wz*dt/2, 1,        wx*dt/2 ]
        #     [wz*dt/2, wy*dt/2 , -wx*dt/2,     1   ]
        F = self.I + (0.5*self.imu_dt*self.quat_differential(gyro))
        return F

    def h(self, quat):
        """
        Measurement model - linearized measurement model function

        :param ndarray quat: predicted quaternion for next time step
        :returns: 
            - h (ndarray) - expected quaternion for next time step
        """
        qw, qx, qy, qz = quat[0][0], quat[1][0], quat[2][0], quat[3][0]
        dcm = quat2dcm(qw, qx, qy, qz)
        a = dcm.T @ self.a_ref # expected accelerometer measurement
        m = dcm.T @ self.m_ref # expected magnetometer measurement
        # when choose 6 axis fusion, only accelerometer data considered
        #       [ax*(0.5-qy**2-qz**2) + ay*(qw*qz+qx*qy)     + az*(qx*qz-qw*qy)    ]
        # H = 2*[ax*(qx*qy-qw*qz)     + ay*(0.5-qx**2-qz**2) + az*(qw*qx+qy*qz)    ]
        #       [ax*(qw*qy+qx*qz)     + ay*(qy*qz-qw*qx)     + az*(0.5-qx**2-qy**2)]

        # when choose 6 axis fusion, both accelerometer and magnetometer data considered
        #       [ax*(0.5-qy**2-qz**2) + ay*(qw*qz+qx*qy)     + az*(qx*qz-qw*qy)    ]
        #       [ax*(qx*qy-qw*qz)     + ay*(0.5-qx**2-qz**2) + az*(qw*qx+qy*qz)    ]
        # H = 2*[ax*(qw*qy+qx*qz)     + ay*(qy*qz-qw*qx)     + az*(0.5-qx**2-qy**2)]
        #       [mx*(0.5-qy**2-qz**2) + my*(qw*qz+qx*qy)     + mz*(qx*qz-qw*qy)    ]
        #       [mx*(qx*qy-qw*qz)     + my*(0.5-qx**2-qz**2) + mz*(qw*qx+qy*qz)    ]
        #       [mx*(qw*qy+qx*qz)     + my*(qy*qz-qw*qx)     + mz*(0.5-qx**2-qy**2)]
        if self.axis == 6:
            h = 2*a
        elif self.axis == 9:
            h = 2*np.vstack((a,m))
        return h

    def H(self, quat):
        """
        Measurement model - Jacobian of linearized measurement model function

        :param ndarray quat: predicted quaternion for next time step
        :returns: 
            - H (ndarray) - Discrete time measurement matrix
        """
        ax, ay, az = self.a_ref[0][0], self.a_ref[1][0], self.a_ref[2][0]
        mx, my, mz = self.m_ref[0][0], self.m_ref[1][0], self.m_ref[2][0]
        qw, qx, qy, qz = quat[0][0], quat[1][0], quat[2][0], quat[3][0]
        # when choose 6 axis fusion, only accelerometer data considered
        #       [ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
        # H = 2*[-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
        #       [ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz]])

        # when choose 6 axis fusion, both accelerometer and magnetometer data considered
        #       [ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
        #       [-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
        # H = 2*[ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz],
        #       [ mx*qw + my*qz - mz*qy, mx*qx + my*qy + mz*qz, -mx*qy + my*qx - mz*qw, -mx*qz + my*qw + mz*qx],
        #       [-mx*qz + my*qw + mz*qx, mx*qy - my*qx + mz*qw,  mx*qx + my*qy + mz*qz, -mx*qw - my*qz + mz*qy],
        #       [ mx*qy - my*qx + mz*qw, mx*qz - my*qw - mz*qx,  mx*qw + my*qz - mz*qy,  mx*qx + my*qy + mz*qz]])
        if self.axis == 6:
            H = 2*np.array([[ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
                            [-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
                            [ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz]])
        elif self.axis == 9:
            H = 2*np.array([[ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
                            [-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
                            [ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz],
                            [ mx*qw + my*qz - mz*qy, mx*qx + my*qy + mz*qz, -mx*qy + my*qx - mz*qw, -mx*qz + my*qw + mz*qx],
                            [-mx*qz + my*qw + mz*qx, mx*qy - my*qx + mz*qw,  mx*qx + my*qy + mz*qz, -mx*qw - my*qz + mz*qy],
                            [ mx*qy - my*qx + mz*qw, mx*qz - my*qw - mz*qx,  mx*qw + my*qz - mz*qy,  mx*qx + my*qy + mz*qz]])
        return H
