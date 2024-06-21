#pragma once
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "../lib/orientation.h"

using namespace std;
using namespace Eigen;

Vector3d acc2eul(float ax, float ay, float az, string nav);
Vector4d acc2quat(float ax, float ay, float az, string nav);

Vector3d accmag2eul(float ax, float ay, float az, float mx, float my, float mz, string nav);
Vector4d accmag2quat(float ax, float ay, float az, float mx, float my, float mz, string nav);

Vector3d ENU2NED(float E, float N, float U);
Vector3d NED2ENU(float N, float E, float D);

Matrix3d skew_symmetric(float x, float y, float z);