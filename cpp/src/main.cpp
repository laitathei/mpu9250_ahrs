#include <iostream>
#include <memory>
#include "../lib/utils.h"
#include "../lib/mpu9250.h"
#include "../lib/madgwick.h"
#include "../lib/mahony.h"
#include "../lib/ekf.h"

using namespace std;

int main(int argc, char **argv)
{
    string nav_frame = "NED"; // ENU/NED
    int axis = 9;
    float hz = 100;
    bool calibration = false;
    // unique_ptr<ahrs> ahrs_ptr = make_unique<Madgwick>(axis, 1, nav_frame);
    // unique_ptr<ahrs> ahrs_ptr = make_unique<Mahony>(axis, 0.1, 0, nav_frame);
    // unique_ptr<ahrs> ahrs_ptr = make_unique<EKF>(axis, Vector3d(pow(0.3,2), pow(0.5,2), pow(0.8,2)), nav_frame);
    MPU9250 imu(nav_frame, axis, hz, calibration);
    imu.initialization();
    imu.start_thread(nullptr); // turn off ahrs filter
    // imu.start_thread(move(ahrs_ptr)); // turn on ahrs filter
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