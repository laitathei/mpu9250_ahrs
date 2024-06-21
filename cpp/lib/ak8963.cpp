#include <math.h>
#include <sstream>
#include <vector>
#include "ak8963.h"
#include "transformation.h"
#include "utils.h"

void AK8963::who_am_i()
{
    int value;
    value = this->read_8bit_register(WIA);
    cout << "The register value is 0x" << hex << value << endl;
    if (value == 0x48){
        cout << "It is AK8963 default value" << endl;
    }
    else{
        cout << "It is not AK8963 default value" << endl;
        throw invalid_argument("AK8963 not found");
    }
}

void AK8963::config_AK8963(int mag_parameter=16)
{
    this->set_mode("fuse rom access", mag_parameter);
    this->get_adjust_mag();
    this->set_mode("power down", mag_parameter);
    this->set_mode("continuous measure 2", mag_parameter);
    this->get_status();
}

void AK8963::get_status()
{
    int ST1_value, bit_0, bit_1;
    ST1_value = this->read_8bit_register(ST1);
    bit_0 = ST1_value & 1; // int("00000001", 2)
    bit_1 = ST1_value & 2; // int("00000010", 2)
    if (bit_0 == 0){
        cout << "Ready in measurement data register or ST2 register" << endl;
    }
    else if (bit_0 == 1){
        cout << "Ready in single measurement mode or self-test mode" << endl;
    }
    else{
        throw invalid_argument("AK8963 status 1 register bit 0 error");
    }

    if (bit_1 == 0){
        cout << "Ready in measurement data register or ST2 register" << endl;
    }
    else if (bit_1 == 2){
        cout << "Data overrun" << endl;
    }
    else{
        throw invalid_argument("AK8963 status 1 register bit 1 error");
    }
}

void AK8963::get_adjust_mag()
{
    int asax, asay, asaz;
    cout << "Read sensitivity adjustment value" << endl;
    asax = this->read_8bit_register(ASAX);
    asay = this->read_8bit_register(ASAY);
    asaz = this->read_8bit_register(ASAZ);

    this->adjustment_x = (((asax-128)*0.5/128)+1);
    this->adjustment_y = (((asay-128)*0.5/128)+1);
    this->adjustment_z = (((asaz-128)*0.5/128)+1);
}

mag_calib AK8963::mag_calibration(float s)
{
    float mx, my, mz;
    Vector3d mag;
    Matrix4d error_matrix;
    Vector4d error_vector;
    if ((s > 0) and (this->calibration == true))
    {
        stringstream ss;
        Vector3d total_gyro, current_gyro;
        total_gyro << 0, 0, 0;
        ss << "Please move the IMU in slow motion in all possible directions, the calibration process takes " << s << "s";
        cout << ss.str();
        cin.ignore(); // Press enter with nothing
        MatrixXd calibration = MatrixXd(int(s*this->hz),4);
        VectorXd target = VectorXd(int(s*this->hz));
        for (int i=0;i<(s*this->hz);i++)
        {
            mag = this->get_mag();
            mx = mag[0];
            my = mag[1];
            mz = mag[2];

            calibration(i,0) = mx;
            calibration(i,1) = my;
            calibration(i,2) = mz;
            calibration(i,3) = 1;
            target[i] = pow(mx,2)+pow(my,2)+pow(mz,2);
            time_sleep(0, 1/this->hz);
        }
        
        error_matrix = calibration.transpose() * calibration;
        error_vector = error_matrix.inverse().eval() * calibration.transpose() * target;

        this->mx_bias = 0.5*error_vector[0];
        this->my_bias = 0.5*error_vector[1];
        this->mz_bias = 0.5*error_vector[2];
        
        this->mag_bias << this->mx_bias, this->my_bias, this->mz_bias;
        this->mag_scale << this->mx_scale, this->my_scale, this->mz_scale;
        this->mag_misalignment << this->mxy_mis, this->mxz_mis, this->myx_mis, this->myz_mis, this->mzx_mis, this->mzy_mis;
        this->mag_strength = pow((error_vector[3]+pow(this->mx_bias,2)+pow(this->my_bias,2)+pow(this->mz_bias,2)),0.5);
    }
    mag_calib result;
    result.scale = this->mag_scale;
    result.bias = this->mag_bias;
    result.misalignment = this->mag_misalignment;
    result.strength = this->mag_strength;
    cout << "Finish magnetometer calibration" << endl;
    return result;
}

Vector3d AK8963::get_mag()
{
    Vector3d mag;
    int ST2_value, bit_3, bit_4;
    float mx, my, mz;

    mx = this->read_raw_data(HXH, HXL)*this->mag_fs;
    my = this->read_raw_data(HYH, HYL)*this->mag_fs;
    mz = this->read_raw_data(HZH, HZL)*this->mag_fs;

    // sensitivity adjustment
    mx = mx*this->adjustment_x;
    my = my*this->adjustment_y;
    mz = mz*this->adjustment_z;

    ST2_value = this->read_8bit_register(ST2);
    bit_3 = ST2_value & 8; // int("00001000", 2)
    bit_4 = ST2_value & 16; // int("00010000", 2)

    // if (bit_3 == 0){
    //     cout << "Ready in measurement data register or ST2 register" << endl;
    // }
    // else if (bit_3 == 8){
    //     cout << "Magnetic sensor overflow occurred" << endl;
    // }
    // else{
    //     throw invalid_argument("AK8963 status 2 register bit 3 error");    
    // }

    // if (bit_4 == 0){
    //     cout << "Magnetic sensor in 14-bit coding" << endl;
    // }
    // else if (bit_4 == 16){
    //     cout << "Magnetic sensor in 16-bit coding" << endl;
    // }
    // else{
    //     throw invalid_argument("AK8963 status 2 register bit 4 error");    
    // }

    if (this->nav_frame == "ENU")
    {
        mag = NED2ENU(mx, my, mz);
        mx = mag[0];
        my = mag[1];
        mz = mag[2]; 
    }

    // magnetometer model: calibrated measurement = (matrix)*(raw measurement - bias)
    if (this->calibration == false)
    {
        mx = (this->mx_scale * mx + this->mxy_mis * my + this->mxz_mis * mz) - this->mx_bias;
        my = (this->myx_mis * mx + this->my_scale * my + this->myz_mis * mz) - this->my_bias;
        mz = (this->mzx_mis * mx + this->mzy_mis * my + this->mz_scale * mz) - this->mz_bias;
    }

    mag << mx, my, mz;
    return mag;
}

void AK8963::set_mode(string mode, int bit)
{
    int value;
    if (mode == "power down"){
        value = 0x00; // 00000000
    }
    else if (mode == "single measure"){
        value = 0x01; // 00000001
    }
    else if (mode == "continuous measure 1"){
        value = 0x02; // 8Hz, 00000010
    }
    else if (mode == "continuous measure 2"){
        value = 0x06; // 100Hz, 00000110
    }
    else if (mode == "external trigger measurement"){
        value = 0x04; // 8Hz, 00000100
    }
    else if (mode == "self test"){
        value = 0x08; // 00001000
    }
    else if (mode == "fuse rom access"){
        value = 0x0F; // 00001111
    }
    else{
        throw invalid_argument("Prohibit mode coding");        
    }

    if (bit == 14){
        value += 0;
    }
    else if (bit == 16){
        value += 16;
    }
    else{
        throw invalid_argument("Wrong bit coding");
    }

    this->mag_fs = 4912*2/pow(2, bit);
    cout << "Set AK8963 to " << mode << " mode" << endl;
    wiringPiI2CWriteReg8(this->i2c, CNTL, value);
}

int AK8963::read_raw_data(int high_register, int low_register)
{
    int high=-1, low=-1, unsigned_value, signed_value;
    while (high == -1) {
        high = wiringPiI2CReadReg8(this->i2c, high_register);
        if (high != -1) {
            break;
        }
    }
    while (low == -1) {
        low = wiringPiI2CReadReg8(this->i2c, low_register);
        if (low != -1) {
            break;
        }
    }

    // Megre higher bytes and lower bytes data
    unsigned_value = (high << 8) + low;

    // Calculate the unsigned int16 range to signed int16 range
    if ((unsigned_value >= 32768) and (unsigned_value < 65536)){
        signed_value = -(~(unsigned_value - 65536)+1);
    }
    else if ((unsigned_value >= 0) and (unsigned_value < 32768)){
        signed_value = unsigned_value;
    }
    return signed_value;
}

int AK8963::read_8bit_register(int single_register)
{
    int data=-1;
    while (data == -1) {
        data = wiringPiI2CReadReg8(this->i2c, single_register);
        if (data != -1) {
            break;
            return data;
        }
    }
}