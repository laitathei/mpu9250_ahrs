#include <iostream>
#include <fstream>
#include <cmath>
#include <map>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "../lib/utils.h"
#include "../lib/orientation.h"
#include "../lib/transformation.h"
#include "../lib/mpu9250.h"

using namespace std;
using namespace Eigen;
using namespace YAML;

int main(int argc, char **argv)
{
    string nav_frame = "NED"; // ENU/NED
    int axis = 9;
    float hz = 100;
    bool calibration = false;
    MPU9250 imu(nav_frame, axis, hz, calibration);
    imu.initialization();
    imu.start_thread();
    clock_t start = clock();
    clock_t last = clock();
    while(1)
    {
        start = clock();
        float dt = float(start - last);
        time_sleep(dt, 1/imu.hz);
        cout << "" << endl;
        cout << "temp: " << imu.temp << endl;
        cout << "nav_frame: " << imu.nav_frame << endl;
        cout << "ax ay az: " << imu.ax << " " << imu.ay << " " << imu.az << endl;
        cout << "gx gy gz: " << imu.gx << " " << imu.gy << " " << imu.gz << endl;
        cout << "mx my mz: " << imu.mx << " " << imu.my << " " << imu.mz << endl;
        cout << "roll pitch yaw: " << imu.roll << " " << imu.pitch << " " << imu.yaw << endl;
        cout << "w x y z: " << imu.w << " " << imu.x << " " << imu.y << " " << imu.z << endl;
        last = clock();
        cout << "hz: " << 1/(float(last - start)/CLOCKS_PER_SEC) << endl;
    }
    return 0;
}