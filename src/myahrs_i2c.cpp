#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "myahrs_plus.h"


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "myahrs_i2c");

  ROS_INFO("Initialization OK!\n");
  ros::spin();

  return 0;
}
