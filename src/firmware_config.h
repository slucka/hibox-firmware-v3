// #ifndef LINO_BASE_CONFIG_H
// #define LINO_BASE_CONFIG_H

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER      // 4WD robot
// #define LINO_BASE ACKERMANN       // Car-like steering robot w/ 2 motors
// #define LINO_BASE ACKERMANN1      // Car-like steering robot w/ 1 motor
// #define LINO_BASE MECANUM         // Mecanum drive robot

//uncomment the motor driver you're using
// #define USE_L298_DRIVER
// #define USE_BTS7960_DRIVER
// #define USE_ESC
// #define USE_TB6612FNG_DRIVER

//uncomment the IMU you're using
// #define USE_GY85_IMU
// #define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
#define USE_MPU9250_IMU

#define LINO_DEBUG 1

#define K_P 0.6 // P constant
#define K_I 0.3 // I constant
#define K_D 0.5 // D constant

//define your robot' specs here
#define MAX_RPM 330                 // motor's maximum RPM
//#define MAX_RPM 150               // motor's maximum RPM

//#define COUNTS_PER_REV 400        // wheel encoder's no of ticks per rev
//#define COUNTS_PER_REV 650        // wheel encoder's no of ticks per rev
//#define COUNTS_PER_REV 1650        // wheel encoder's no of ticks per rev
#define COUNTS_PER_REV 1500        // wheel encoder's no of ticks per rev

//#define WHEEL_DIAMETER 0.062      // wheel's diameter in meters
#define WHEEL_DIAMETER 0.066      // wheel's diameter in meters
//#define PWM_BITS 8                // PWM Resolution of the microcontroller
#define PWM_BITS 8                // PWM Resolution of the microcontroller
//#define LR_WHEELS_DISTANCE 0.170  // distance between left and right wheels
#define LR_WHEELS_DISTANCE 0.195  // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN
#define MAX_STEERING_ANGLE 0.415  // max steering angle. This only applies to Ackermann steering

// #ifdef USE_TB6612FNG_DRIVER
  #define MOTOR_DRIVER TB6612FNG

  #define MOTOR1_PWM  5
  #define MOTOR1_IN_A 3
  #define MOTOR1_IN_B 99

  #define MOTOR2_PWM  6
  #define MOTOR2_IN_A 4
  #define MOTOR2_IN_B 99

  #define MOTOR3_PWM  99
  #define MOTOR3_IN_A 99
  #define MOTOR3_IN_B 99

  #define MOTOR4_PWM  99
  #define MOTOR4_IN_A 99
  #define MOTOR4_IN_B 99

  #define PWM_MAX  pow(2, PWM_BITS) - 1
  #define PWM_MIN (pow(2, PWM_BITS) - 1) * -1
// #endif 

// #define STEERING_PIN 7

// #endif
