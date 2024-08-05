#include <iostream>
#include <memory>
#include <eigen3/Eigen/Dense>
#include "../lib/mpu9250.h"
#include "../lib/madgwick.h"
#include "../lib/mahony.h"
#include "../lib/ekf.h"
#include "../lib/visualization.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    GLint window_width = 1080;
    GLint window_height = 720;
    int window_hz = 100;
    string nav_frame = "ENU"; // ENU/NED
    int axis = 9;
    bool calibration = false;
    // unique_ptr<ahrs> ahrs_ptr = make_unique<Madgwick>(axis, 1, nav_frame);
    // unique_ptr<ahrs> ahrs_ptr = make_unique<Mahony>(axis, 0.1, 0, nav_frame);
    // unique_ptr<ahrs> ahrs_ptr = make_unique<EKF>(axis, Vector3d(pow(0.3,2), pow(0.5,2), pow(0.8,2)), nav_frame);
    MPU9250 imu(nav_frame, axis, window_hz, calibration);
    imu.initialization();
    imu.start_thread(nullptr); // turn off ahrs filter
    // imu.start_thread(move(ahrs_ptr)); // turn on ahrs filter
    imu_viewer viewer(window_width, window_height, window_hz, nav_frame);
    viewer.run(imu);
    return 0;
}