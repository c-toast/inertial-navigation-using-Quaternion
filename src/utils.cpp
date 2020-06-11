#include<eigen3/Eigen/Dense>

#include"utils.h"
#include<unistd.h>
#include<string>

using namespace std;

#define RAW_FILE_NAME "./raw/raw_data"
#define RESULT_FILE_NAME "./result/result"
int FREQUENCY=200;

string GetRawFileNameStr(){
	for(int i=1;;i++){
		char res[50];
		sprintf(res,"%s%d",RAW_FILE_NAME,i);
		if(access(res,F_OK)!=0){
			return string(res);
		}
	}
}

string GetResultFileNameStr(){
	for(int i=1;;i++){
		char res[50];
		sprintf(res,"%s%d.txt",RESULT_FILE_NAME,i);
		if(access(res,F_OK)!=0){
			return res;
		}
	}
}

Matrix<float, 4, 1> Quaternion_to_Vect(Eigen::Quaternionf q)
{
    Matrix<float, 4, 1> vector;
    vector(0) = q.w();
    vector.block<3, 1>(1, 0) = q.vec();

    return vector;
}

//C_b^n
void Euler2Rotation(Matrix<float,3,3> &C,Matrix<float,3,1> &euler){
    float yaw=euler(0.0);
    float pitch=euler(1.0);
    float roll=euler(2.0);
    C(0,0)=cos(roll)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw);
    C(0,1)=cos(pitch)*sin(yaw);
    C(0,2)=sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw);

    C(1,0)=-cos(roll)*sin(yaw)+sin(roll)*sin(pitch)*cos(yaw);
    C(1,1)=cos(pitch)*cos(yaw);
    C(1,2)=-sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw);
    
    C(2,0)=-sin(roll)*cos(pitch);
    C(2,1)=sin(pitch);
    C(2,2)=cos(roll)*cos(pitch);
}

Matrix<float, 3, 1> Rotation_to_Euler(Matrix<float, 3, 3> Rotation)
{
    Matrix<float, 3, 1> Euler;

    //Euler(0) = atan2(Rotation(1, 2), Rotation(2, 2));//yaw
    Euler(0) = atan(Rotation(0, 1)/Rotation(1, 1));//yaw
    Euler(1) = asin(Rotation(2, 1));//pitch
    //Euler(2) = atan2(Rotation(0, 1), Rotation(0, 0));//roll
    Euler(2) = atan(-Rotation(2, 0)/Rotation(2, 2));//roll

    return Euler;
}

Matrix<float,3,1> Quaternion_to_Euler(Eigen::Quaternionf q)
{
    Matrix<float,3,1> Euler;
    float q0=q.w();
    float q1=q.x();
    float q2=q.y();
    float q3=q.z();
	Euler(0) = atan2f(2*(q1*q2-q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
	Euler(1) = asinf(2*(q2*q3+q0*q1));
	Euler(2) = atan2f(-2*(q1*q3-q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
	return Euler;
}

void Quaternion_to_Rotation(Eigen::Quaternionf &q,Matrix<float,3,3> &C){
    float q0=q.w();
    float q1=q.x();
    float q2=q.y();
    float q3=q.z();

    C(0,0)=q0*q0+q1*q1-q2*q2-q3*q3;
    C(0,1)=2*(q1*q2-q0*q3);
    C(0,2)=2*(q1*q3+q0*q2);

    C(1,0)=2*(q1*q2+q0*q3);
    C(1,1)=q0*q0-q1*q1+q2*q2-q3*q3;
    C(1,2)=2*(q2*q3-q0*q1);
    
    C(2,0)=2*(q1*q3-q0*q2);
    C(2,1)=2*(q2*q3+q0*q1);
    C(2,2)=q0*q0-q1*q1-q2*q2+q3*q3;
}

Quaternionf Euler_to_Quaternion(Matrix<float,3,1> Euler) 
{
    Quaternionf quaternion;

    quaternion = Eigen::AngleAxisf(Euler(0), Eigen::Vector3f::UnitZ())

    * Eigen::AngleAxisf(Euler(1), Eigen::Vector3f::UnitX())

    *Eigen::AngleAxisf(Euler(2), Eigen::Vector3f::UnitY());

    return quaternion;
}