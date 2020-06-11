#pragma once

#include<eigen3/Eigen/Dense>
#include<cstdio>

#define LOGD(args...) fprintf(stdout,"D:");fprintf(stdout, ##args); putchar('\n')
#define LOGI(args...) fprintf(stdout,"I:");fprintf(stdout, ##args); putchar('\n')
#define LOGE(args...) fprintf(stdout,"E:");fprintf(stdout, ##args); putchar('\n')

std::string GetRawFileNameStr();

std::string GetResultFileNameStr();

using namespace Eigen;

Matrix<float, 4, 1> Quaternion_to_Vect(Eigen::Quaternionf q);

Matrix<float, 3, 1> Rotation_to_Euler(Matrix<float, 3, 3> Rotation);

inline float GetRoll(float roll) {
    return roll * 57.29578f;
}

inline float GetPitch(float pitch) {
    return pitch * 57.29578f;
}

inline float GetYaw(float yaw) {
    return yaw * 57.29578f;// + 180.0f;
}

template <typename T>
void PrintMatrix(const T& matrix,int row,int column){
    for(int i=0;i<row;i++){
        for(int j=0;j<column;j++){
            printf("%f ",matrix(i,j));
        }
        printf("\n");
    }
}

Matrix<float,3,1> Quaternion_to_Euler(Eigen::Quaternionf q);

Quaternionf Euler_to_Quaternion(Matrix<float,3,1> Euler);

void Quaternion_to_Rotation(Eigen::Quaternionf &q,Matrix<float,3,3> &C);

void Euler2Rotation(Matrix<float,3,3> &C,Matrix<float,3,1> &euler);