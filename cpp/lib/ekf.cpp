#include <cmath>
#include "ekf.h"
#include "transformation.h"

void EKF::init_quat(float w, float x, float y, float z)
{
    this->est_quat << w, x, y, z;
}

Vector4d EKF::run(Vector3d acc, Vector3d gyr, Vector3d mag, float hz)
{
    if (acc.rows() != 3 || acc.cols() != 1) {
        throw invalid_argument("acc shape must be (3,1)");
    } else if (gyr.rows() != 3 || gyr.cols() != 1) {
        throw invalid_argument("gyr shape must be (3,1)");
    } else if (mag.rows() != 3 || mag.cols() != 1) {
        throw invalid_argument("mag shape must be (3,1)");
    }
    this->imu_hz = hz;
    this->imu_dt = 1/this->imu_hz;
    this->acc = acc;
    this->gyr = gyr;
    this->mag = mag;
    if (this->axis == 6){ // 6 axis fusion (gyroscope and accelerometer)
        this->est_quat = this->gyro_acc_fusion();
    }
    else if (this->axis == 9){ // 9 axis fusion (gyroscope, accelerometer and magnetometer)
        this->est_quat = this->gyro_acc_mag_fusion();
    }
    return this->est_quat;
}

Vector4d EKF::gyro_acc_fusion()
{
    float qw, qx, qy, qz;
    float ax, ay, az;
    float gx, gy, gz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    Vector3d mag = this->mag;
    float a_norm = acc.norm();
    float m_norm = mag.norm();
    qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
    if (a_norm > 0)
    {
        Vector3d a = acc/a_norm; // normalize acceleration vector
        // ----- Prediction -----
        MatrixXd f = this->f(gyr);
        MatrixXd F = this->F(gyr);
        MatrixXd W(4, 3);
        W << -qx, -qy, -qz,
              qw, -qz,  qy,
              qz,  qw, -qx,
             -qy,  qx,  qw;
        W = 0.5*this->imu_dt*W;
        MatrixXd Q = this->gyro_noise * W * W.transpose();  // Process Noise Covariance Matrix
        P = F * this->P * F.transpose() + Q;  // Predicted Covariance Matrix
        // ----- Correction -----
        Vector3d z = a;
        Vector3d y = this->h(f); // Expected Measurement Function
        Vector3d v = z - y; // Innovation (Measurement Residual)
        MatrixXd H = this->H(f); // Discreted Measurement Matrix
        Matrix3d S = H * P * H.transpose() + this->R; // Measurement Prediction Covariance
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman Gain 
        this->P = (this->I - K * H) * P; // Updated Covariance Matrix
        this->est_quat = f + K * v; // Corrected State
        this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    }
    return this->est_quat;
}

Vector4d EKF::gyro_acc_mag_fusion()
{
    float qw, qx, qy, qz;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    Vector3d acc = this->acc;
    Vector3d gyr = this->gyr;
    Vector3d mag = this->mag;
    float a_norm = acc.norm();
    float m_norm = mag.norm();
    qw = this->est_quat[0], qx = this->est_quat[1], qy = this->est_quat[2], qz = this->est_quat[3];
    if ((a_norm > 0) and (m_norm > 0))
    {
        Vector3d a = acc/a_norm; // normalize acceleration vector
        Vector3d m = mag/m_norm; // normalize magnetometer vector
        // ----- Prediction -----
        MatrixXd f = this->f(gyr);
        MatrixXd F = this->F(gyr);
        MatrixXd W(4, 3);
        W << -qx, -qy, -qz,
              qw, -qz,  qy,
              qz,  qw, -qx,
             -qy,  qx,  qw;
        W = 0.5*this->imu_dt*W;
        MatrixXd Q = this->gyro_noise * W * W.transpose();  // Process Noise Covariance Matrix
        P = F * this->P * F.transpose() + Q;  // Predicted Covariance Matrix
        // ----- Correction -----
        VectorXd vstack(a.rows() + m.rows(), a.cols());
        vstack << a, m;
        VectorXd z = vstack;
        VectorXd y = this->h(f); // Expected Measurement Function
        VectorXd v = z - y; // Innovation (Measurement Residual)
        MatrixXd H = this->H(f); // Discreted Measurement Matrix
        MatrixXd S = H * P * H.transpose() + this->R; // Measurement Prediction Covariance
        MatrixXd K = P * H.transpose() * S.inverse(); // Kalman Gain 
        this->P = (this->I - K * H) * P; // Updated Covariance Matrix
        this->est_quat = f + K * v; // Corrected State
        this->est_quat = this->est_quat/this->est_quat.norm(); // normalize quaternion vector
    }
    return this->est_quat;
}

MatrixXd EKF::quat_differential(Vector3d gyro)
{
    //          [0 , -wx, -wy, -wz]
    // matrix = [wx,  0 ,  wz, -wy]
    //          [wy, -wz,  0 ,  wx]
    //          [wz,  wy, -wx,  0 ]
    float wx, wy, wz;
    wx = gyro[0], wy = gyro[1], wz = gyro[2];
    MatrixXd matrix = MatrixXd::Zero(4, 4);
    Matrix3d skew = skew_symmetric(wx, wy, wz);
    Vector4d vector(0, wx, wy, wz);
    matrix.col(0) = vector;
    matrix.row(0) = -vector.transpose();
    matrix.block<3, 3>(1, 1) = skew.transpose();
    return matrix;
}

MatrixXd EKF::f(Vector3d gyro)
{
    //     [qw - wx*qx*dt/2 - wy*qy*dt/2 - wz*qz*dt/2]
    // f = [qx + wx*qw*dt/2 - wy*qz*dt/2 + wz*qy*dt/2]
    //     [qy + wx*qz*dt/2 + wy*qw*dt/2 - wz*qx*dt/2]
    //     [qz - wx*qy*dt/2 + wy*qx*dt/2 + wz*qw*dt/2]
    MatrixXd matrix = this->I + (0.5*this->imu_dt*this->quat_differential(gyro));
    MatrixXd f = matrix * this->est_quat;
    return f;
}

MatrixXd EKF::F(Vector3d gyro)
{
    //     [1      , -wx*dt/2, -wy*dt/2, -wz*dt/2]
    // F = [wx*dt/2,     1   , wz*dt/2,  -wy*dt/2]
    //     [wy*dt/2, -wz*dt/2, 1,        wx*dt/2 ]
    //     [wz*dt/2, wy*dt/2 , -wx*dt/2,     1   ]
    MatrixXd F = this->I + (0.5*this->imu_dt*this->quat_differential(gyro));
    return F;
}

VectorXd EKF::h(Vector4d quat)
{
    MatrixXd h;
    float qw, qx, qy, qz;
    qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    Matrix3d dcm = quat2dcm(qw, qx, qy, qz);
    Vector3d a = dcm.transpose()*this->a_ref;
    Vector3d m = dcm.transpose()*this->m_ref;
    // when choose 6 axis fusion, only accelerometer data considered
    //       [ax*(0.5-qy**2-qz**2) + ay*(qw*qz+qx*qy)     + az*(qx*qz-qw*qy)    ]
    // H = 2*[ax*(qx*qy-qw*qz)     + ay*(0.5-qx**2-qz**2) + az*(qw*qx+qy*qz)    ]
    //       [ax*(qw*qy+qx*qz)     + ay*(qy*qz-qw*qx)     + az*(0.5-qx**2-qy**2)]

    // when choose 6 axis fusion, both accelerometer and magnetometer data considered
    //       [ax*(0.5-qy**2-qz**2) + ay*(qw*qz+qx*qy)     + az*(qx*qz-qw*qy)    ]
    //       [ax*(qx*qy-qw*qz)     + ay*(0.5-qx**2-qz**2) + az*(qw*qx+qy*qz)    ]
    // H = 2*[ax*(qw*qy+qx*qz)     + ay*(qy*qz-qw*qx)     + az*(0.5-qx**2-qy**2)]
    //       [mx*(0.5-qy**2-qz**2) + my*(qw*qz+qx*qy)     + mz*(qx*qz-qw*qy)    ]
    //       [mx*(qx*qy-qw*qz)     + my*(0.5-qx**2-qz**2) + mz*(qw*qx+qy*qz)    ]
    //       [mx*(qw*qy+qx*qz)     + my*(qy*qz-qw*qx)     + mz*(0.5-qx**2-qy**2)]
    if (this->axis == 6){
        h = 2*a;
    }
    else if (this->axis == 9){
        MatrixXd vstack(a.rows() + m.rows(), a.cols());
        vstack << a, m;
        h = 2 * vstack;
    }
    return h;
}

MatrixXd EKF::H(Vector4d quat)
{
    MatrixXd H;
    float ax, ay, az;
    float mx, my, mz;
    float qw, qx, qy, qz;
    ax = this->a_ref[0], ay = this->a_ref[1], az = this->a_ref[2];
    mx = this->m_ref[0], my = this->m_ref[1], mz = this->m_ref[2];
    qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
    // when choose 6 axis fusion, only accelerometer data considered
    //       [ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
    // H = 2*[-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
    //       [ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz]])

    // when choose 6 axis fusion, both accelerometer and magnetometer data considered
    //       [ ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx],
    //       [-ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw,  ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy],
    // H = 2*[ ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx,  ax*qw + ay*qz - az*qy,  ax*qx + ay*qy + az*qz],
    //       [ mx*qw + my*qz - mz*qy, mx*qx + my*qy + mz*qz, -mx*qy + my*qx - mz*qw, -mx*qz + my*qw + mz*qx],
    //       [-mx*qz + my*qw + mz*qx, mx*qy - my*qx + mz*qw,  mx*qx + my*qy + mz*qz, -mx*qw - my*qz + mz*qy],
    //       [ mx*qy - my*qx + mz*qw, mx*qz - my*qw - mz*qx,  mx*qw + my*qz - mz*qy,  mx*qx + my*qy + mz*qz]])
    if (this->axis == 6) {
        H = MatrixXd(3, 4);
        H << ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx,
            -ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw, ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy,
             ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx, ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz;
    } 
    else if (this->axis == 9) {
        H = MatrixXd(6, 4);
        H << ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz, -ax*qy + ay*qx - az*qw, -ax*qz + ay*qw + az*qx,
            -ax*qz + ay*qw + az*qx, ax*qy - ay*qx + az*qw, ax*qx + ay*qy + az*qz, -ax*qw - ay*qz + az*qy,
             ax*qy - ay*qx + az*qw, ax*qz - ay*qw - az*qx, ax*qw + ay*qz - az*qy, ax*qx + ay*qy + az*qz,
             mx*qw + my*qz - mz*qy, mx*qx + my*qy + mz*qz, -mx*qy + my*qx - mz*qw, -mx*qz + my*qw + mz*qx,
            -mx*qz + my*qw + mz*qx, mx*qy - my*qx + mz*qw, mx*qx + my*qy + mz*qz, -mx*qw - my*qz + mz*qy,
             mx*qy - my*qx + mz*qw, mx*qz - my*qw - mz*qx, mx*qw + my*qz - mz*qy, mx*qx + my*qy + mz*qz;
    }
    return 2 * H;
}