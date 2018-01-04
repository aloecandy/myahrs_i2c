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

std::string parent_frame_id_="base_link";
std::string frame_id_="imu_link";




unsigned char ret = -1;

class PubNode
{
    public:
    ros::NodeHandle nh_;
    ros::Publisher imu_data_pub_;
    tf::TransformBroadcaster broadcaster_;
    
    PubNode(){
        imu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 1);
    }
    ~PubNode(){

    }
    void publish_msg(){
        static double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
        static double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
        static double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
        static double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
        static double convertor_c    = 1.0;        // for temperature (celsius)
        
        sensor_msgs::Imu imu_data_msg;

        
        double roll, pitch, yaw;
        // original sensor data used the degree unit, convert to radian (see ROS REP103)
        // we used the ROS's axes orientation like x forward, y left and z up
        // so changed the y and z aixs of myAHRS+ board
        roll  = p_euler->roll*convertor_d2r;
        pitch =  -p_euler->pitch*convertor_d2r;
        yaw   = -p_euler->yaw*convertor_d2r;
        tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);
        
        imu_data_msg.linear_acceleration.x = p_sensor->acc_x*convertor_g2a;
        imu_data_msg.linear_acceleration.y = -p_sensor->acc_y*convertor_g2a;
        imu_data_msg.linear_acceleration.z = -p_sensor->acc_z*convertor_g2a;

        imu_data_msg.angular_velocity.x = p_sensor->gyro_x*convertor_d2r;
        imu_data_msg.angular_velocity.y = -p_sensor->gyro_y*convertor_d2r;
        imu_data_msg.angular_velocity.z = -p_sensor->gyro_z*convertor_d2r;

        imu_data_msg.orientation.x = orientation[0];
        imu_data_msg.orientation.y = orientation[1];
        imu_data_msg.orientation.z = orientation[2];
        imu_data_msg.orientation.w = orientation[3];
        
        imu_data_msg.linear_acceleration_covariance[0] =
        imu_data_msg.linear_acceleration_covariance[4] =
        imu_data_msg.linear_acceleration_covariance[8] = 0.026831*0.026831;
        imu_data_msg.angular_velocity_covariance[0] =
        imu_data_msg.angular_velocity_covariance[4] =
        imu_data_msg.angular_velocity_covariance[8] = 0.002428*0.002428;
        imu_data_msg.orientation_covariance[0] =
        imu_data_msg.orientation_covariance[4] =
        imu_data_msg.orientation_covariance[8] = 0.002143*0.002143;

        imu_data_msg.header.stamp     = ros::Time::now();
        imu_data_msg.header.frame_id     = "imu_link";

    
        imu_data_pub_.publish(imu_data_msg);
        broadcaster_.sendTransform(tf::StampedTransform(tf::Transform(tf::createQuaternionFromRPY(roll,pitch,yaw),
            tf::Vector3(0.0,0.0,0.0)),
            ros::Time::now(),parent_frame_id_, frame_id_));
    }

};




int main(int argc, char* argv[])
{
        

    
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
    PubNode pubNode;

    
    ROS_INFO("Initialization OK!\n");
    
    ros::Rate loop_rate(100);
    while(ros::ok()){
        read();
        print_imu(p_sensor,p_euler,p_quat);
        pubNode.publish_msg();


        loop_rate.sleep();
    }
  
  ros::spin();
  free(p_sensor);
  free(p_euler);
  free(p_quat);

  return 0;
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
    i2c_myahrs_raw_sensor_read(p_sensor);
    //i2c_myahrs_cal_sensor_read(p_sensor);
    
    i2c_myahrs_euler_read(p_euler);

    i2c_myahrs_quat_read(p_quat);
}