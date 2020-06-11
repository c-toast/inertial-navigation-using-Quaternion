#include <ros/ros.h>
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <string>
#include <fstream>
#include <serialPort/gyro_acc.h>//erase this #include and include new head file of msg
#include <math.h>
#include <signal.h>
#include "utils.h"

struct
{
    float x, y, z;
} acc;

struct
{
    float x, y, z;
} gyro;

serial::Serial ser; //the object of serial port
//do not use this msg definition, redefine new msg or directly use ros imu msg
serialPort::gyro_acc valid_data_msg;
extern int FREQUENCY;//defined in utils.cpp

float ReadFloatFromStr(std::string &s, int start_index)
{
    union {
        char c[4];
        float f;
    } tmp;
    memcpy(tmp.c, s.c_str() + start_index, 4);
    return tmp.f;
}

//this function should be rewrite according to the protocol of imu serial port
bool ReadAccordingToDataFormat(std::string &s)
{
    char head[3] = {0x5a, 0x5a,0x00};     
    int head_index = s.find(head);     
    if (head_index == -1)
    {
        return false;
    }
    if(s.size()-head_index<59){
        return false;
    }
    char sum=0;
    for(int i=2;i<58;i++){
        sum+=s[head_index+i];
        sum=sum&(0xff);
    }
    if(sum!=s[head_index+58]){
        s.erase(0, head_index+59);
        return false;
    }

    gyro.x = 0.0174533 * ReadFloatFromStr(s, head_index + 2);
    gyro.y = 0.0174533 * ReadFloatFromStr(s, head_index + 6);
    gyro.z = 0.0174533 * ReadFloatFromStr(s, head_index + 10);
    acc.x = ReadFloatFromStr(s, head_index + 14)*9.81;
    acc.y = ReadFloatFromStr(s, head_index + 18)*9.81;
    acc.z = ReadFloatFromStr(s, head_index + 22)*9.81;

    s.erase(0, head_index+59); //erase the string that have been identified
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::Publisher read_pub = nh.advertise<serialPort::gyro_acc>("read", 5000);
    try
    {
        //set the property of serial port
        ser.setPort("/dev/IMU102N");
        ser.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }
    //set frequency 
    ros::Rate loop_rate(FREQUENCY);
    std::string result; //the string to receive data
    while (ros::ok())
    {
        if (ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port\n");
            std::string buffer=ser.read(ser.available());
            result.append(buffer);
            
            while(ReadAccordingToDataFormat(result))
            {
                valid_data_msg.gyro_x = gyro.x;
                valid_data_msg.gyro_y = gyro.y;
                valid_data_msg.gyro_z = gyro.z;
                valid_data_msg.acc_x = acc.x;
                valid_data_msg.acc_y = acc.y;
                valid_data_msg.acc_z = acc.z;
                valid_data_msg.time = 1;
                read_pub.publish(valid_data_msg);
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }   
}
