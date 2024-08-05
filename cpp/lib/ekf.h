#pragma once
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "ahrs.h"
#include "orientation.h"

using namespace std;
using namespace Eigen;

class EKF : public ahrs
{
    /**
     * @brief Extended Kalman filter is one of the AHRS filter dealing with sensor gaussian noise 

     * @param axis: axis data for fusion
     * @param noise: gyroscope, accelerometer, magnetometer gaussian noise
     * @param nav_frame: navigation frame
     */

    private:
        int axis;
        float gyro_noise, accel_noise, mag_noise;
        float imu_hz, imu_dt;
        float declination, inclination, north_intensity, east_intensity, vertical_intensity, horizontal_intensity, total_intensity;
        string nav_frame;
        Vector3d acc, gyr, mag, a_ref, m_ref;
        Vector4d est_quat;
        MatrixXd R;
        Matrix4d I, P;

    public:
        EKF(int axis, Vector3d noise, string nav_frame="NED") {
            // Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
            // Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

            // algorithm parameter
            this->axis = axis; // 6 or 9
            this->nav_frame = nav_frame; // ENU or NED
            this->gyro_noise = noise[0]; // increase noise means gyroscope measurement not accurate, decrease noise means gyroscope measurement accurate
            this->accel_noise = noise[1]; //increae noise means accelerometer measurement not accurate, decrease noise means accelerometer measurement accurate
            this->mag_noise = noise[2]; // increase noise means magnetometer measurement not accurate, decrease noise means magnetometer measurement accurate
            VectorXd v;
            if (this->axis == 6){
                v = VectorXd::Constant(1, accel_noise);
            }
            else if (this->axis == 9){
                v = VectorXd(2);
                v << accel_noise, mag_noise;
            }
            R = v.replicate(3, 1).asDiagonal();
            I = Matrix4d::Identity();
            P = Matrix4d::Identity();

            // Hong Kong geomagnetic field parameter
            // http://www.geomag.bgs.ac.uk/data_service/models_compass/wmm_calc.html

            this->declination = -3.404; // declination angle (degree)
            this->inclination = 33.739; // inclination angle (degree)
            this->north_intensity = 37794; // north intensity (nT)
            this->east_intensity = -2248; // east intensity (nT)
            this->horizontal_intensity = 37861; // horizontal intensity (nT)
            this->vertical_intensity = 25287; // vertical intensity (nT)
            this->total_intensity = 45529; // total intensity (nT)

            this->a_ref << 0, 0, 1; // Gravitational Reference Vector (due to my accelerometer definition)
            if (this->nav_frame == "ENU"){
                this->m_ref << 0, cos(this->inclination * M_PI / 180.0), -sin(this->inclination * M_PI / 180.0); // Magnetic Reference Vector
            }
            else if (this->nav_frame == "NED")
            {
                this->m_ref << cos(this->inclination * M_PI / 180.0), 0, sin(this->inclination * M_PI / 180.0); // Magnetic Reference Vector
            }
            float m_ref_norm = m_ref.norm();
            this->m_ref << this->m_ref/m_ref_norm;

            // Check parameter
            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED"))
            {
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }

            if ((this->axis != 6) and (this->axis != 9))
            {
                throw invalid_argument("Axis must be 6 or 9");
            }

            cout << "Extended Kalman filter in use" << endl;
        }
        void init_quat(float w, float x, float y, float z) override;
        Vector4d run(Vector3d accl, Vector3d gyro, Vector3d mag, float hz) override;
        Vector4d gyro_acc_fusion();
        Vector4d gyro_acc_mag_fusion();
        MatrixXd quat_differential(Vector3d gyro);
        MatrixXd f(Vector3d gyro);
        MatrixXd F(Vector3d gyro);
        VectorXd h(Vector4d quat);
        MatrixXd H(Vector4d quat);
};