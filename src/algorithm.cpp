#include <serialPort/gyro_acc.h>
#include <ros/ros.h>
#include<algorithm>
#include"utils.h"
#include"attitude_estimator.h"

extern int FREQUENCY;

void read_callback(const serialPort::gyro_acc::ConstPtr &msg)
{
    static float ax_average=0;
    static float ay_average=0;
    static float az_average=0;
    static float current_time=0;
    static int counter=0;
    static AttitudeEstimator estimator(FREQUENCY);

    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, heading;
    ax = msg->acc_x;
    ay = msg->acc_y;
    az = msg->acc_z;
    gx=msg->gyro_x;
    gy=msg->gyro_y;
    gz=msg->gyro_z;

    //initialize the attitude by the average of first 1000 acceleration 
    {
        int acc_data_num=1000;
        if(counter<acc_data_num){
            counter++;
            ax_average+=ax;
            ay_average+=ay;
            az_average+=az;
            if(counter==acc_data_num){
                ax_average/=acc_data_num;
                ay_average/=acc_data_num;
                az_average/=acc_data_num;
                estimator.InitAttitude(ax_average,ay_average,az_average);
            }
            return ;

        }
    }
    //estimate attitude, velocity, position
    estimator.EstimateAttitude(gx,gy,gz,ax,ay,az);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "algorithm_node");
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("read", 5000, read_callback);
    ros::Rate loop_rate(FREQUENCY);
    printf("initial succeed");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}