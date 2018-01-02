/*
*********************************************************************************************************
*                                              MY_AHRS+_C
*********************************************************************************************************
*/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "myahrs_plus.h"

/*
*********************************************************************************************************
*                                   DEFINE MACROS & GLOBAL VARIABLE
*********************************************************************************************************
*/
#define I2C_DEV_FILENAME    "/dev/i2c-1"
static int imu_fd;

/*
*********************************************************************************************************
*                                           MY_AHRS+_FUNCTION
*********************************************************************************************************
*/

/* 
* imu setup
* int i2c_myahrs_setup(void)
* 입력 값 : 없음
* 반환 값 : 성공 0 / 실패 -1
* 설명 : MYAHRS+ 디바이스의 접근성 확보.
*/
int i2c_myahrs_setup(void)
{
    int ret=0;

    //i2c 디바이스파일을 오픈
    if((imu_fd = open(I2C_DEV_FILENAME, O_RDWR )) < 0){
        perror("open()");
        return -1;
    }

    //접근할 디바이스의 슬레이브 주소를 설정합니다.
    if((ret = ioctl(imu_fd, I2C_SLAVE, AHRS_SLAVE_ADDRESS)) < 0){
        perror("ioctl");
        close(imu_fd);
        exit(1);
    }

    return ret;
}

/* 
* myahrs+ 상태 확인.
* int I2C_MYAHRS_READ(int fd, unsigned char type, ahrs_data_t * ahrs_data)
* 입력 값 : 없음
* 반환 값 : 성공 0 이상 / 실패 -1
* 설명 : 올바르게 imu_setup이 완료 되어있는지 확인을 위해 i2c write/read를 통한 imu slave name 확인.
*/
int i2c_myahrs_state_check(void)
{
    unsigned char temp_buf[5]={0,};
    unsigned char reg = 0;
    int ret=0;

    reg = REG_WHO_AM_I;
    if((ret = write(imu_fd,&reg,1))<0){
        perror("write");
        return -1;
    }

    if((ret = read(imu_fd,temp_buf,4)) < 0){
        perror("read");
        return -1;
    }

    // 0xB1이 출력되어야 정상이다. 
    printf("WHO AM I : 0x%02X\r\n",temp_buf[0]);
    printf("FIRMWARE VERSION : %02d.%02d \r\n",temp_buf[1],temp_buf[2]);
    printf("REG_STATUS : 0x%02X\r\n",temp_buf[3]);
    
    if(temp_buf[0] != AHRS_WHO_AM_I){
        printf("Doesn't match imu name!!");
        return -1;
    }

    return ret;
}

/* 
* myahrs+ raw sensor 데이터 리드
* int i2c_myahrs_raw_sensor_read(ahrs_sensor_t * sensor_data)
* 입력 값 : ahrs_sensor_t 구조체 포인터
* 반환 값 : 성공 read한 값 / 실패 -1
* 설명 : 보정되기 전의 9축 sensor 데이터를 읽어와 저장.
*/
int i2c_myahrs_raw_sensor_read(ahrs_sensor_t * sensor_data)
{
    unsigned char temp_buf[19]={0,};
    unsigned char reg = 0;
    int ret=0;
    ahrs_sensor_t *p_sensor_data = (ahrs_sensor_t *)sensor_data;
    
#ifdef IMU_PRINT
    printf("\r\nread I2C MY AHRS+ RAW SENSOR DATA. \r\n");
#endif

    reg = REG_I_ACC_X_LOW;
    if((ret = write(imu_fd,&reg,1))<0){
        perror("write");
        return -1;
    }

    if((ret = read(imu_fd,temp_buf,18)) < 0){
        perror("read");
        return -1;
    }

    /*비트 연산 후 singed short(16bit)로 캐스팅 후 곱,나눗셈 연산을 위해 float 캐스팅*/
    p_sensor_data->acc_x    = (float)(signed short)((temp_buf[1]<<8)|temp_buf[0]) * 16.0 / 32767.;
    p_sensor_data->acc_y    = (float)(signed short)((temp_buf[3]<<8)|temp_buf[2]) * 16.0 / 32767.;
    p_sensor_data->acc_z    = (float)(signed short)((temp_buf[5]<<8)|temp_buf[4]) * 16.0 / 32767.;
    
    p_sensor_data->gyro_x   = (float)(signed short)((temp_buf[7]<<8)|temp_buf[6]) * 2000. / 32767.;
    p_sensor_data->gyro_y   = (float)(signed short)((temp_buf[9]<<8)|temp_buf[8]) * 2000. / 32767.;
    p_sensor_data->gyro_z   = (float)(signed short)((temp_buf[11]<<8)|temp_buf[10]) * 2000. / 32767.;
    
    p_sensor_data->magnet_x = (float)(signed short)((temp_buf[13]<<8)|temp_buf[12]) * 0.3;
    p_sensor_data->magnet_y = (float)(signed short)((temp_buf[15]<<8)|temp_buf[14]) * 0.3;
    p_sensor_data->magnet_z = (float)(signed short)((temp_buf[17]<<8)|temp_buf[16]) * 0.3;
    
#ifdef IMU_PRINT
    printf("ACC[g]:%.2f,%.2f,%.2f\r\n", p_sensor_data->acc_x, p_sensor_data->acc_y, p_sensor_data->acc_z);
    printf("GYRO[deg/s]:%.2f,%.2f,%.2f\r\n", p_sensor_data->gyro_x, p_sensor_data->gyro_y, p_sensor_data->gyro_z);
    printf("MAG[uT]:%.2f,%.2f,%.2f\r\n", p_sensor_data->magnet_x, p_sensor_data->magnet_y, p_sensor_data->magnet_z);
#endif

    return ret;
}

/* 
* myahrs+ cal sensor 데이터 리드
* int i2c_myahrs_cal_sensor_read(ahrs_sensor_t * sensor_data)
* 입력 값 : ahrs_sensor_t 구조체 포인터
* 반환 값 : 성공 read한 값 / 실패 -1
* 설명 : 보정 후의 9축 sensor 데이터를 읽어와 저장.
*/
int i2c_myahrs_cal_sensor_read(ahrs_sensor_t * sensor_data)
{
    unsigned char temp_buf[19]={0,};
    unsigned char reg = 0;
    int ret=0;
    ahrs_sensor_t *p_sensor_data = (ahrs_sensor_t *)sensor_data;

#ifdef IMU_PRINT
    printf("\r\nread I2C MY AHRS+ CALCULATE SENSOR DATA. \r\n");
#endif

    reg = REG_C_ACC_X_LOW;
    if((ret = write(imu_fd,&reg,1))<0){
        perror("write");
        return -1;
    }

    if((ret = read(imu_fd,temp_buf,18)) < 0){
        perror("read");
        return -1;
    }

    /*비트 연산 후 singed short(16bit)로 캐스팅 후 곱,나눗셈 연산을 위해 float 캐스팅*/
    p_sensor_data->acc_x    = (float)(signed short)((temp_buf[1]<<8)|temp_buf[0]) * 16.0 / 32767.;
    p_sensor_data->acc_y    = (float)(signed short)((temp_buf[3]<<8)|temp_buf[2]) * 16.0 / 32767.;
    p_sensor_data->acc_z    = (float)(signed short)((temp_buf[5]<<8)|temp_buf[4]) * 16.0 / 32767.;
    
    p_sensor_data->gyro_x   = (float)(signed short)((temp_buf[7]<<8)|temp_buf[6]) * 2000. / 32767.;
    p_sensor_data->gyro_y   = (float)(signed short)((temp_buf[9]<<8)|temp_buf[8]) * 2000. / 32767.;
    p_sensor_data->gyro_z   = (float)(signed short)((temp_buf[11]<<8)|temp_buf[10]) * 2000. / 32767.;
    
    p_sensor_data->magnet_x = (float)(signed short)((temp_buf[13]<<8)|temp_buf[12]) * 0.3;
    p_sensor_data->magnet_y = (float)(signed short)((temp_buf[15]<<8)|temp_buf[14]) * 0.3;
    p_sensor_data->magnet_z = (float)(signed short)((temp_buf[17]<<8)|temp_buf[16]) * 0.3;
    
#ifdef IMU_PRINT
    printf("ACC[g]:%.2f,%.2f,%.2f\r\n", p_sensor_data->acc_x, p_sensor_data->acc_y, p_sensor_data->acc_z);
    printf("GYRO[deg/s]:%.2f,%.2f,%.2f\r\n", p_sensor_data->gyro_x, p_sensor_data->gyro_y, p_sensor_data->gyro_z);
    printf("MAG[uT]:%.2f,%.2f,%.2f\r\n", p_sensor_data->magnet_x, p_sensor_data->magnet_y, p_sensor_data->magnet_z);
#endif
   
    return ret;
}

/* 
* myahrs+ 오일러 각도 데이터 리드
* int i2c_myahrs_euler_read(ahrs_euler_t * euler_data)
* 입력 값 : ahrs_euler_t 구조체 포인터
* 반환 값 : 성공 read한 값 / 실패 -1
* 설명 : 보정 후의 9축 sensor 데이터를 이용해 전처리된 오일러 각도를 읽어와 저장.
*/
int i2c_myahrs_euler_read(ahrs_euler_t * euler_data)
{
    unsigned char temp_buf[7]={0,};
    unsigned char reg = 0;
    int ret=0;
    ahrs_euler_t *p_euler_data = (ahrs_euler_t *)euler_data;

#ifdef IMU_PRINT
    printf("\r\nread I2C MY AHRS+ EULER ANGLE DATA. \r\n");
#endif

    reg = REG_ROLL_LOW;
    if((ret = write(imu_fd,&reg,1))<0){
        perror("write");
        return -1;
    }

    if((ret = read(imu_fd,temp_buf,6)) < 0){
        perror("read");
        return -1;
    }

    /*비트 연산 후 singed short(16bit)로 캐스팅 후 곱,나눗셈 연산을 위해 float 캐스팅*/
    p_euler_data->roll     = (float)(signed short)((temp_buf[1]<<8)|temp_buf[0]) * 180. / 32767.;
    p_euler_data->pitch    = (float)(signed short)((temp_buf[3]<<8)|temp_buf[2]) * 180. / 32767.;
    p_euler_data->yaw      = (float)(signed short)((temp_buf[5]<<8)|temp_buf[4]) * 180. / 32767.;

#ifdef IMU_PRINT
    printf("R/P/Y[deg]:%.2f,%.2f,%.2f\r\n", p_euler_data->roll,p_euler_data->pitch, p_euler_data->yaw);
#endif

    return ret;
}

/* 
* myahrs+ 쿼터니언 데이터 리드
* int i2c_myahrs_quat_read(ahrs_quaternian_t * quat_data)
* 입력 값 : ahrs_quaternian_t 구조체 포인터
* 반환 값 : 성공 read한 값 / 실패 -1
* 설명 : 보정 후의 9축 sensor 데이터를 이용해 전처리된 쿼터니언 각도를 읽어와 저장.
*/
int i2c_myahrs_quat_read(ahrs_quaternian_t * quat_data)
{
    unsigned char temp_buf[9]={0,};
    unsigned char reg = 0;
    int ret=0;
    ahrs_quaternian_t *p_quat_data = (ahrs_quaternian_t *)quat_data;

#ifdef IMU_PRINT
    printf("\r\nread I2C MY AHRS+ QUATERNIAN DATA. \r\n");
#endif
    
    reg = REG_QUATERNION_X_LOW;
    if((ret = write(imu_fd,&reg,1))<0){
        perror("write");
        return -1;
    }

    if((ret = read(imu_fd,temp_buf,8)) < 0){
        perror("read");
        return -1;
    }

    /*비트 연산 후 singed short(16bit)로 캐스팅 후 곱,나눗셈 연산을 위해 float 캐스팅*/
    p_quat_data->x            = (float)(signed short)((temp_buf[1]<<8)|temp_buf[0]) / 32767.;
    p_quat_data->y            = (float)(signed short)((temp_buf[3]<<8)|temp_buf[2]) / 32767.;
    p_quat_data->z            = (float)(signed short)((temp_buf[5]<<8)|temp_buf[4]) / 32767.;
    p_quat_data->w            = (float)(signed short)((temp_buf[7]<<8)|temp_buf[6]) / 32767.;

#ifdef IMU_PRINT
    printf("Quaternian:%.4f,%.4f,%.4f,%.4f\r\n", p_quat_data->x, p_quat_data->y, p_quat_data->z, p_quat_data->w);
#endif
   
    return ret;
}

/* 
* myahrs+ 데이터 리드
* int i2c_read(unsigned char * buf, unsigned int size, unsigned char reg)
* 입력 값 : buf ==> 읽어온 데이터의 저장 포인터
*         size ==> 읽어올 데이터의 길이
*         reg ==> 읽어올 i2c 시작 주소
* 반환 값 : 성공 read한 값 / 실패 -1
* 설명 : 사용자 임의로 i2c 디바이스의 주소에 접근하여 데이터를 읽어옴 
*/
int i2c_read(unsigned char * buf, unsigned int size, unsigned char reg)
{
    unsigned char * temp_buf = buf;
    unsigned char tmp_reg = reg;
    int ret=0;

    if(size < 1){
        printf("unvaild size!! \n");
        return -1;
    }else{
        printf("\r\nread I2C MY AHRS+ DATA. \r\n");
    
        if((ret = write(imu_fd,&tmp_reg,1)) < 0){
            perror("write");
            return -1;
        }
    
        if((ret = read(imu_fd,temp_buf,size)) < 0){
            perror("read");
            return -1;
        }

        return ret;
    } 
}