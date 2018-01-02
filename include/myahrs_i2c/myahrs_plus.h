/*

*********************************************************************************************************

*                                              MY_AHRS+_H

*

* ##raspberry pi 3 i2c slave - Myahrs+ device site : http://www.withrobot.com/myahrs_plus/

*

* device repository : https://github.com/withrobot/myAHRS_plus

*

* ##Connectivity myahrs+ & raspberry pi3   myahrs + //  raspberry pi3 

* j3-3(I2C_SCL)         --  pin 3(Physical) 

* j3-4(I2C_SDA)         --  pin 5(physical)   

* j4-1(VDD)             --  pin 2,4(physical)   

* j4-10(GND)            --  pin 6,9,14,20,25,30,34(physical)

*

*********************************************************************************************************

*/

#ifndef __MYAHRS_PLUS_H__

#define __MYAHRS_PLUS_H__

/*

* 선택 옵션

* IMU_PRINT : imu에 관련된 읽어온 정보를 프린트.

*/

//#define IMU_PRINT



#define AHRS_SLAVE_ADDRESS      0x20

#define AHRS_WHO_AM_I           0xB1 

#define AHRS_STATUS_READY       0x80        // bit mask for STATUS register 

/*

*********************************************************************************************************

*                                  MY_AHRS+ I2C DEFINE MACROS & VARIABLE

* myAHRS+ I2C 인터페이스

* myAHRS+는 I2C slave로 동작하며, I2C 버스에는 pull-up 저항이 필요하다.

* 일반적으로 4.7k옴을 사용하며, 상황에 따라 1k옴,10k옴의 저항을 사용 가능하다.

* 라즈베리파이의 I2C핀과 GPIO는 기본적으로 내부에 1k옴의 풀업저항을 내장하므로 

* 라즈베리파이와 인터페이스시에는 별도의 풀업저항을 달지않아도 무방하다.

* I2C 포트는 3.3v 전압 레벨에서 동작한다. 5V IO를 사용하는 MCU와 별도의 회로없이 바로 연결해서 사용 가능하다.

* 기타 사양은 다음과 같다.

* I2C Slave address : 7 bit, 0x20

* Data bit : 8bit

* I2C clock speed : Normal mode(100KHz), Fast mode(400KHz)

* 이하의 레지스터는 myAHRS+의 레지스터 주소이다.

*********************************************************************************************************

*/

#define REG_WHO_AM_I            0x01 // 0xB1 myAHRS+임을 의미하는 상수

#define REG_REV_ID_MAJOR        0x02 // 펌웨어 리비전 주 번호

#define REG_REV_ID_MINOR        0x03 // 펌웨어 리비전 부 번호

#define REG_STATUS              0x04 // 0x80 센서의 상태를 나타내며 모든 초기화 과정을 정상적으로 마치면 최상위 비트가 1이 된다. 

/*

* 영점 조정 파라미터로 보정하지 않은 센서 출력값(signed 16bit)을 저장한 레지스터

* low 레지스터에 하위 8bit를, high 레지스터에 상위 8bit를 저장한다.

* 원래 정수이므로 실수로 변환할 필요 없다.

*/

#define REG_I_ACC_X_LOW         0x10 //Acceleration raw 

#define REG_I_ACC_X_HIGH        0x11 //Acceleration raw 

#define REG_I_ACC_Y_LOW         0x12 //Acceleration raw 

#define REG_I_ACC_Y_HIGH        0x13 //Acceleration raw 

#define REG_I_ACC_Z_LOW         0x14 //Acceleration raw 

#define REG_I_ACC_Z_HIGH        0x15 //Acceleration raw 

#define REG_I_GYRO_X_LOW        0x16 //Gyroscope raw data

#define REG_I_GYRO_X_HIGH       0x17 //Gyroscope raw data

#define REG_I_GYRO_Y_LOW        0x18 //Gyroscope raw data

#define REG_I_GYRO_Y_HIGH       0x19 //Gyroscope raw data

#define REG_I_GYRO_Z_LOW        0x1A //Gyroscope raw data

#define REG_I_GYRO_Z_HIGH       0x1B //Gyroscope raw data

#define REG_I_MAGNET_X_LOW      0x1C //Magnetometer raw data

#define REG_I_MAGNET_X_HIGH     0x1D //Magnetometer raw data

#define REG_I_MAGNET_Y_LOW      0x1E //Magnetometer raw data

#define REG_I_MAGNET_Y_HIGH     0x1F //Magnetometer raw data

#define REG_I_MAGNET_Z_LOW      0x20 //Magnetometer raw data

#define REG_I_MAGNET_Z_HIGH     0x21 //Magnetometer raw data

/*

* 보정한 가속도 센서의 출력값(signed 16bit)을 저장한 레지스터. 

* LOW 레지스터에 하위 8bit high레지스터에 상위 8bit를 저장한다. 

* scale factor(16/32767.0)을 곱하여 g 단위의 가속도로 변환한다.

* 가속도(g) = C_ACC * 16 /32767.0

*/

#define REG_C_ACC_X_LOW         0x22 //Calibrated acceleration data 

#define REG_C_ACC_X_HIGH        0x23 //Calibrated acceleration data

#define REG_C_ACC_Y_LOW         0x24 //Calibrated acceleration data

#define REG_C_ACC_Y_HIGH        0x25 //Calibrated acceleration data

#define REG_C_ACC_Z_LOW         0x26 //Calibrated acceleration data

#define REG_C_ACC_Z_HIGH        0x27 //Calibrated acceleration data

/*

* 보정한 각속도 센서의 출력 값(signed 16bit)을 저장한 레지스터.

* Low 레지스터에 하위 8bit를, high레지스터에 상위 8bit를 저장한다.

* scale factor(2000/32767.0)을 곱하여 dps(°/s)단위의 가속도로 변환한다.

* 각속도(dps) = C_GYRO * 2000 / 32767.0

*/

#define REG_C_GYRO_X_LOW        0x28 //Calibrated gyroscope data

#define REG_C_GYRO_X_HIGH       0x29 //Calibrated gyroscope data

#define REG_C_GYRO_Y_LOW        0x2A //Calibrated gyroscope data

#define REG_C_GYRO_Y_HIGH       0x2B //Calibrated gyroscope data

#define REG_C_GYRO_Z_LOW        0x2C //Calibrated gyroscope data

#define REG_C_GYRO_Z_HIGH       0x2D //Calibrated gyroscope data

/*

* 보정한 지자기 센서의 출력 값(signed 16bit)을 저장한 레지스터.

* LOW 레지스터에 하위 8bit를, HIGH레지스터에 상위 8bit를 저장한다.

* 보정한 지자기 센서의 출력값은 물리적인 단위를 상실하므로 굳이 실수로 변환할 필요없다.

* 만약 자기장 측정값이 필요하다면 보정하지 않은 지자기 센서 출력(I_MAGNET_X_LOW ~ HIGH)

* 에 scale factor(0.3)을 곱하면 마이크로T단위의 자기장 측정값을 얻을 수 있다.

* 지자기(마이크로T) = I_MAGNET * 0.3

*/

#define REG_C_MAGNET_X_LOW      0x2E //Calibrated magnetometer data

#define REG_C_MAGNET_X_HIGH     0x2F //Calibrated magnetometer data

#define REG_C_MAGNET_Y_LOW      0x30 //Calibrated magnetometer data

#define REG_C_MAGNET_Y_HIGH     0x31 //Calibrated magnetometer data

#define REG_C_MAGNET_Z_LOW      0x32 //Calibrated magnetometer data

#define REG_C_MAGNET_Z_HIGH     0x33 //Calibrated magnetometer data

/*

* 온도 센서의 출력 값(signed 16bit)을 저장한 레지스터.

* LOW 레지스터에 하위 8bit를, HIGH레지스터에 상위 8bit를 저장한다.

* scale factor(200/32767.0)을 곱하여 °C 단위의 온도로 변환한다.

* 온도(°C) = TEMPERATURE_N * 200 /32767

*/

#define REG_TEMPERATURE_LOW     0x34 //Temperature data

#define REG_TEMPERATURE_HIGH    0x35 //Temperature data

/*

* 오일러각(signed 16bit)을 저장한 레지스터

* LOW 레지스터에 하위 8bit를, HIGH레지스터에 상위 8bit를 저장한다.

* scale factor(180/32767.0)을 곱하여 degree(°)단위의 각도로 변환한다.

* 오일러각(°) =ROLL(PITCH,YAW) * 180 / 32767

*/

#define REG_ROLL_LOW            0x36 //Euler angle 

#define REG_ROLL_HIGH           0x37 //Euler angle 

#define REG_PITCH_LOW           0x38 //Euler angle 

#define REG_PITCH_HIGH          0x39 //Euler angle 

#define REG_YAW_LOW             0x3A //Euler angle 

#define REG_YAW_HIGH            0x3B //Euler angle 

/*

* 쿼터니언(signed 16bit)을 저장한 레지스터

* LOW 레지스터에 하위 8bit를, HIGH레지스터에 상위 8bit를 저장한다.

* scale factor(1/32767.0)을 곱하여 실수로 변환한다.

* 쿼터니언 = X(Y,Z,W) /32767

*/

#define REG_QUATERNION_X_LOW    0x3C //Quaternion 

#define REG_QUATERNION_X_HIGH   0x3D //Quaternion 

#define REG_QUATERNION_Y_LOW    0x3E //Quaternion 

#define REG_QUATERNION_Y_HIGH   0x3F //Quaternion 

#define REG_QUATERNION_Z_LOW    0x40 //Quaternion 

#define REG_QUATERNION_Z_HIGH   0x41 //Quaternion 

#define REG_QUATERNION_W_LOW    0x42 //Quaternion 

#define REG_QUATERNION_W_HIGH   0x43 //Quaternion 



/*

*********************************************************************************************************

*                                           DEFINE STRUCTURE TYPE

* ahrs_sensor_t      : 9축 센서 데이터를 저장할 구조체

* ahrs_euler_t       : 오일러 각도를 저장할 구조체

* ahrs_quaternian_t  : 쿼터니언 각도를 저장할 구조체

*

*********************************************************************************************************

*/

typedef struct

{

    float acc_x;

    float acc_y;

    float acc_z;

    float gyro_x;

    float gyro_y;

    float gyro_z;

    float magnet_x;

    float magnet_y;

    float magnet_z;

} ahrs_sensor_t;



typedef struct

{

    float roll;

    float pitch;

    float yaw;

} ahrs_euler_t;



typedef struct

{

    float x;

    float y;

    float z;

    float w;

} ahrs_quaternian_t;



/*

*********************************************************************************************************

*                                              PREDEFINE FUNCTION

*********************************************************************************************************

*/

int i2c_myahrs_setup(void);

int i2c_myahrs_state_check(void);

int i2c_myahrs_raw_sensor_read(ahrs_sensor_t * sensor_data);

int i2c_myahrs_cal_sensor_read(ahrs_sensor_t * sensor_data);

int i2c_myahrs_euler_read(ahrs_euler_t * euler_data);

int i2c_myahrs_quat_read(ahrs_quaternian_t * quat_data);

int i2c_read(unsigned char * buf, unsigned int size, unsigned char reg);



#endif /*__MYAHRS_PLUS_H__*/