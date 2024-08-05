#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ahrs.h"
#include "orientation.h"

using namespace std;
using namespace Eigen;

class Mahony : public ahrs
{
    /**
     * @brief Mahony is one of the AHRS filter applied with complementary filter

     * @param axis: axis data for fusion
     * @param kp: proportional gain
     * @param ki: integral gain
     * @param nav_frame: navigation frame
     */

    private:
        int axis;
        float ki, kp;
        float imu_hz, imu_dt;
        string nav_frame;
        Vector3d acc, gyr, mag;
        Vector3d gyro_bias;
        Vector4d est_quat;

    public:
        Mahony(int axis, float kp, float ki, string nav_frame="NED") { // Constructor with parameters
            // Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
            // Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

            // algorithm parameter
            this->axis = axis; // 6 or 9
            this->ki = ki; // reduce the steady-state error
            this->kp = kp; // increasing trusts more accelerometers and magnetometers, decreasing trusts more gyroscopes
            this->nav_frame = nav_frame; // ENU or NED

            // Check parameter
            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED"))
            {
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }

            if ((this->axis != 6) and (this->axis != 9))
            {
                throw invalid_argument("Axis must be 6 or 9");
            }

            cout << "Mahony filter in use" << endl;
        }

        void init_quat(float w, float x, float y, float z) override;
        Vector4d run(Vector3d accl, Vector3d gyro, Vector3d mag, float hz) override;
        Vector4d gyro_acc_fusion();
        Vector4d gyro_acc_mag_fusion();
};