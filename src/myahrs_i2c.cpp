#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "myahrs_i2c/myahrs_plus.h"


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

void print_imu(ahrs_sensor_t *sensor, ahrs_euler_t *euler, ahrs_quaternian_t *quat);
void read();
void publish_msg();

ahrs_sensor_t     * p_sensor = (ahrs_sensor_t*)malloc(sizeof(ahrs_sensor_t));
ahrs_euler_t      * p_euler  = (ahrs_euler_t*)malloc(sizeof(ahrs_euler_t));
ahrs_quaternian_t * p_quat   = (ahrs_quaternian_t*)malloc(sizeof(ahrs_quaternian_t));
ros::NodeHandle nh;
ros::Publisher imu_data_pub_;

int main(int argc, char* argv[])
{
    
    
    
    

    unsigned char ret = -1;
    //unsigned char buf[3] ={0,};

    
    //i2c 디바이스파일을 오픈
    if((ret = i2c_myahrs_setup())<0){
        perror("imu setup");
        exit(1);
    }
    else{
        printf("myAHSR+ Device Setup Sucess!\n");
    }
        
        
    if((ret = i2c_myahrs_state_check()) < 0){
        perror("imu state");
        exit(1);
    }
    else{
        printf("myAHSR+ Device Connecting OK!\n");
    }
    
    ros::init(argc, argv, "myahrs_i2c");
    ROS_INFO("Initialization OK!\n");
    
    imu_data_pub_       = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    
    while(true){
        read();
        publish_msg();
    }
  
  ros::spin();
  free(p_sensor);
  free(p_euler);
  free(p_quat);

  return 0;
}
void publish_msg(){
    sensor_msgs::Imu imu_data;
    
    imu_data_msg.linear_acceleration_covariance[0] = p_sensor.acc_x;
    imu_data_msg.linear_acceleration_covariance[1] = p_sensor.acc_y;
    imu_data_msg.linear_acceleration_covariance[2] = p_sensor.acc_z;
    
    imu_data_msg.angular_velocity_covariance[0] = p_euler.roll;
    imu_data_msg.angular_velocity_covariance[1] = p_euler.pitch;
    imu_data_msg.angular_velocity_covariance[2] = p_euler.yaw;

    imu_data_msg.orientation_covariance[0] = p_quat.x;
    imu_data_msg.orientation_covariance[1] = p_quat.y;
    imu_data_msg.orientation_covariance[2] = p_quat.z;
    
    imu_data_msg.header.stamp     = ros::Time::now();
    imu_data_msg.header.frame_id     = "imu_link"

    double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
    double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;
    double orientation_cov         = orientation_stddev_ * orientation_stddev_;
    imu_data_pub_.publish(imu_data_msg);
}
void print_imu(ahrs_sensor_t *sensor, ahrs_euler_t *euler, ahrs_quaternian_t *quat){
    printf("\r\nIMU DATA\r\n");
    printf("ACC[g]:%.2f,%.2f,%.2f\r\n", sensor->acc_x, sensor->acc_y, sensor->acc_z);
    printf("GYRO[deg/s]:%.2f,%.2f,%.2f\r\n", sensor->gyro_x, sensor->gyro_y, sensor->gyro_z);
    printf("MAG[uT]:%.2f,%.2f,%.2f\r\n", sensor->magnet_x, sensor->magnet_y, sensor->magnet_z);
    printf("R/P/Y[deg]:%.2f,%.2f,%.2f\r\n", euler->roll,euler->pitch, euler->yaw);
    printf("Quaternian:%.4f,%.4f,%.4f,%.4f\r\n", quat->x, quat->y, quat->z, quat->w);
}

void read(){
    if((ret = i2c_myahrs_raw_sensor_read(p_sensor))<0)
        break;
    usleep(100000);
    
    if((ret = i2c_myahrs_cal_sensor_read(p_sensor))<0)
        break;
    usleep(100000);

    if((ret = i2c_myahrs_euler_read(p_euler))<0)
        break;
    usleep(100000);

    if((ret = i2c_myahrs_quat_read(p_quat))<0)
        break;
    usleep(100000);
}