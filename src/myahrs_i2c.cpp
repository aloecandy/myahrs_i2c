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


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "myahrs_i2c");

  ROS_INFO("Initialization OK!\n");
int j =0;
    unsigned char ret = -1;
    unsigned char buf[3] ={0,};

    ahrs_sensor_t     * p_sensor = (ahrs_sensor_t*)malloc(sizeof(ahrs_sensor_t));
    ahrs_euler_t      * p_euler  = (ahrs_euler_t*)malloc(sizeof(ahrs_euler_t));
    ahrs_quaternian_t * p_quat   = (ahrs_quaternian_t*)malloc(sizeof(ahrs_quaternian_t));
    
    //i2c 디바이스파일을 오픈
    if((ret = i2c_myahrs_setup())<0){
        perror("imu setup");
        exit(1);
    }else
        printf("myAHSR+ Device Setup Sucess!\n");
        
    if((ret = i2c_myahrs_state_check()) < 0){
        perror("imu state");
        exit(1);
    }else 
        printf("myAHSR+ Device Connecting OK!\n");

    while(j++ != 10){
        
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

        #if 0
        //Read test 
        ret = i2c_read(buf,2,REG_TEMPERATURE_LOW);
        printf("buf : 0x%x 0x%x \t",buf[0],buf[1]);
        printf("temp : %.2f \n",(float)(signed short)(buf[1]<<8|buf[0])*200/32767);
        #endif
        print_imu(p_sensor,p_euler,p_quat);
    }
    
    free(p_sensor);
    free(p_euler);
    free(p_quat);

  ros::spin();

  return 0;
}

void print_imu(ahrs_sensor_t *sensor, ahrs_euler_t *euler, ahrs_quaternian_t *quat)
{
    printf("\r\nIMU DATA\r\n");
    printf("ACC[g]:%.2f,%.2f,%.2f\r\n", sensor->acc_x, sensor->acc_y, sensor->acc_z);
    printf("GYRO[deg/s]:%.2f,%.2f,%.2f\r\n", sensor->gyro_x, sensor->gyro_y, sensor->gyro_z);
    printf("MAG[uT]:%.2f,%.2f,%.2f\r\n", sensor->magnet_x, sensor->magnet_y, sensor->magnet_z);
    printf("R/P/Y[deg]:%.2f,%.2f,%.2f\r\n", euler->roll,euler->pitch, euler->yaw);
    printf("Quaternian:%.4f,%.4f,%.4f,%.4f\r\n", quat->x, quat->y, quat->z, quat->w);
}