#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ahrs.h"
#include "orientation.h"

using namespace std;
using namespace Eigen;

class Madgwick : public ahrs
{
    /**
     * @brief Madgwick is one of the AHRS filter applied with gradient descent technique

     * @param axis: axis data for fusion
     * @param gain: 6/9 axis fusion gain
     * @param nav_frame: navigation frame
     */

    private:
        int axis;
        float gain, imu_hz, imu_dt;
        string nav_frame;
        Vector3d acc, gyr, mag;
        Vector4d est_quat;

    public:
        Madgwick(int axis, float gain, string nav_frame="NED") {
            // Body frame is front(X)-right(Y)-down(Z), Navigation frame is NED
            // Body frame is front(Y)-right(X)-down(Z), Navigation frame is ENU

            // algorithm parameter
            // Increasing the gain makes the filter respond quickly to changes, but it also causes the filter to become sensitive to noise.
            // Reducing the gain makes the filter take more time to converge, which means a delay in calculating the attitude
            // increase gain means trust more accelerometers and magnetometers, decreasing means trust more gyroscopes
            this->axis = axis; // 6 or 9
            this->gain = gain; // 6 axis default gain 0.033, 9 axis default gain 0.041
            this->nav_frame = nav_frame;

            // Check parameter
            if ((this->nav_frame != "ENU") and (this->nav_frame != "NED"))
            {
                throw invalid_argument("Navigation frame should be either ENU or NED");
            }

            if ((this->axis != 6) and (this->axis != 9))
            {
                throw invalid_argument("Axis must be 6 or 9");
            }

            cout << "Madgwick filter in use" << endl;
        }

        void init_quat(float w, float x, float y, float z) override;
        Vector4d run(Vector3d accl, Vector3d gyro, Vector3d mag, float hz) override;
        Vector4d gyro_acc_fusion();
        Vector4d gyro_acc_mag_fusion();
};