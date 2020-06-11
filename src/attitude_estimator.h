#pragma once

#include<eigen3/Eigen/Dense>

using namespace Eigen;

class AttitudeEstimator{
    public:
    Matrix<float,3,3> rotation_matrix_;
    Quaternionf quat_;
    
    float delta_t_;
    Matrix<float,3,1> velocity_;
    Matrix<float,3,1> position_;
    Matrix<float,3,1> euler_;//yaw,pitch,roll
    Matrix<float,3,1> acc_;//coordinate n

    void InitAttitude(float ax, float ay, float az);

    void EstimateAttitude(float gx, float gy, float gz, float ax, float ay, float az);

    AttitudeEstimator(int frequency){delta_t_=1.0/frequency;};

    void CalculatePoi_Vel(float ax, float ay, float az);
};

