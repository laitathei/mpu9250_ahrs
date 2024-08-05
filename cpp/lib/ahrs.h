#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class ahrs{
    public:
        ahrs(){};
        virtual void init_quat(float w, float x, float y, float z){};
        virtual Vector4d run(Vector3d accl, Vector3d gyro, Vector3d mag, float hz){};
        virtual ~ahrs(){};  // Ensure the destructor is virtual and defined
};