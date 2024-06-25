#include <iostream>
#include "../lib/mpu9250.h"
#include "../lib/visualization.h"

int main(int argc, char **argv)
{
    GLint window_width = 1080;
    GLint window_height = 720;
    int window_hz = 100;
    string nav_frame = "NED"; // ENU/NED
    int axis = 9;
    bool calibration = false;
    MPU9250 imu(nav_frame, axis, window_hz, calibration);
    imu.initialization();
    imu.start_thread();
    imu_viewer viewer(window_width, window_height, window_hz, nav_frame);
    viewer.run(imu);
    return 0;
}