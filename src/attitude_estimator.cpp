#include<eigen3/Eigen/Dense>
#include<cmath>
#include<cstdarg>
#include<algorithm>
#include<vector>
#include"utils.h"

#include"attitude_estimator.h"

using namespace std;


void AttitudeEstimator::InitAttitude(float ax, float ay, float az){
    //when imu is static, the acceleration is the projection of gravity in imu coordinate
    //we can know the attitude with this relation between acceleration and gravity
    float norm=sqrt(ax*ax+ay*ay+az*az);
    float roll_=atan2(-ax,az);
    float pitch_=asinf(ay/norm);
    float yaw_=0;
    euler_(0,0)=yaw_;euler_(1,0)=pitch_;euler_(2,0)=roll_;
    Euler2Rotation(rotation_matrix_,euler_);
    quat_=Euler_to_Quaternion(euler_);
    
    acc_(0,0)=ax;acc_(1,0)=ay;acc_(2,0)=az;
    acc_=rotation_matrix_*acc_;
     
    velocity_(0,0)=0;velocity_(1,0)=0;velocity_(2,0)=0;
    position_(0,0)=0;position_(1,0)=0;position_(2,0)=0;
}



void AttitudeEstimator::CalculatePoi_Vel(float ax, float ay, float az){
    Matrix<float,3,1> g_vec;
    acc_(0,0)=ax;acc_(1,0)=ay;acc_(2,0)=az;
    g_vec(0,0)=0;g_vec(1,0)=0;g_vec(2,0)=9.81;
    acc_=rotation_matrix_*acc_-g_vec;
    position_=position_+velocity_*delta_t_+0.5*acc_*delta_t_*delta_t_;
    velocity_=velocity_+acc_*delta_t_;
}

void AttitudeEstimator::EstimateAttitude(float gx, float gy, float gz, float ax, float ay, float az){
    float theta_x=gx*delta_t_;
    float theta_y=gy*delta_t_;
    float theta_z=gz*delta_t_;
    Matrix<float,4,4> theta_matrix;
    float half_norm_theta=sqrt(theta_x*theta_x+theta_y*theta_y+theta_z*theta_z)/2;
    theta_matrix(0,0)=0; theta_matrix(0,1)=-theta_x; theta_matrix(0,2)=-theta_y; theta_matrix(0,3)=-theta_z;
    theta_matrix(1,0)=theta_x; theta_matrix(1,1)=0; theta_matrix(1,2)=theta_z; theta_matrix(1,3)=-theta_y;
    theta_matrix(2,0)=theta_y; theta_matrix(2,1)=-theta_z; theta_matrix(2,2)=0; theta_matrix(2,3)=theta_x;
    theta_matrix(3,0)=theta_z; theta_matrix(3,1)=theta_y; theta_matrix(3,2)=-theta_x; theta_matrix(3,3)=0;   

    //calculate the quaternion vector with first order Runge-Kutta
    Matrix<float, 4, 1> q_vec=Quaternion_to_Vect(quat_);
    Matrix4f I=Matrix4f::Identity();
    //Matrix<float, 4, 1> new_q_vec = (I*cos(half_norm_theta)+0.5*theta_matrix/half_norm_theta*sin(half_norm_theta))*q_vec;
    Matrix<float, 4, 1> new_q_vec = (I+theta_matrix*0.5)*q_vec;////

    //transform the vector into Quaternionf
    //notice that in the augument of Quaternionf constructor, the last one of the vector is w.
    float tmp_w=new_q_vec(0,0);
    new_q_vec.block<3,1>(0,0)=new_q_vec.block<3,1>(1,0);
    new_q_vec(3,0)=tmp_w;
    quat_=Quaternionf(new_q_vec);
    
    //renew ratation matrix and yaw, pitch, roll
    quat_.normalize();
    Quaternion_to_Rotation(quat_,rotation_matrix_);
    euler_=Quaternion_to_Euler(quat_);

    CalculatePoi_Vel(ax,ay,az);
    LOGI("ax is %f, ay is %f, az is %f (navigation coordinate)",acc_(0),acc_(1),acc_(2));
    LOGI("yaw is %f, pitch is %f, roll is %f",GetYaw(euler_(0)),GetPitch(euler_(1)),GetRoll(euler_(2)));
    LOGI("vx is %f,vy is %f,vz is %f",velocity_(0,0),velocity_(1,0),velocity_(2,0));
    LOGI("px is %f,py is %f,pz is %f",position_(0,0),position_(1,0),position_(2,0));    
    LOGI(" ");
}
