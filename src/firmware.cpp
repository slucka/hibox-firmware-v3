#include <Arduino.h>

#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "firmware_config.h"
#include "Motor.h"
#include "Kinematics.h"
//#include "PID.h"
//#include <PIDController.h>
#include "Imu.h"
#include "PinChangeInterrupt.h"

#define IMU_PUBLISH_RATE 20 //hz
#define COMMAND_RATE 20     //hz
#define DEBUG_RATE 5


Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

// PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
// PIDController motor1pid;
// PIDController motor2pid;

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist &cmd_msg);
void PIDCallback(const lino_msgs::PID &pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B A14
#define ENCODER_RIGHT_A 7
#define ENCODER_RIGHT_B A15

long odom_left_pulse = 0;
long odom_right_pulse = 0;
unsigned long odom_prev_update_time;
long odom_prev_left_pulse = 0;
long odom_prev_right_pulse = 0;
int odom_counts_per_rev = COUNTS_PER_REV;

char buffer[50];
char buffer_temp[20];

void EncoderLeftCallBack(void)
{
    int p1 = digitalRead(ENCODER_LEFT_A);
    int p2 = digitalRead(ENCODER_LEFT_B);
    int p3 = digitalRead(ENCODER_LEFT_A);

    int p = p1 + p2 + p3;
    if (p == 0 || p == 3)
    {
        odom_left_pulse++; // Moving reverse
    }
    else
    {
        odom_left_pulse--; // Moving forward
    }
    
    //sprintf(buffer, "left_pulse: %d", odom_left_pulse);
    //nh.loginfo(buffer);
}

void EncoderRightCallBack(void)
{

    int p1 = digitalRead(ENCODER_RIGHT_A);
    int p2 = digitalRead(ENCODER_RIGHT_B);
    int p3 = digitalRead(ENCODER_RIGHT_A);

    int p = p1 + p2 + p3;
    if (p == 0 || p == 3)
    {
        odom_right_pulse--; // Moving forward
    }
    else
    {
        odom_right_pulse++; // Moving reverse
    }
    
    //sprintf(buffer, "right_pulse: %d", odom_right_pulse);
    //nh.loginfo(buffer);
}

void getRPM(int &rpm_motor1, int &rpm_motor2)
{
    long current_left_pulse = odom_left_pulse;
    long current_right_pulse = odom_right_pulse;

    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - odom_prev_update_time;

    //convert the time from milliseconds to minutes
    double delta_ticks;
    double dtm = (double)dt / 60000;

    delta_ticks = current_left_pulse - odom_prev_left_pulse;
    rpm_motor1 = ((delta_ticks / odom_counts_per_rev) / dtm);

    delta_ticks = current_right_pulse - odom_prev_right_pulse;
    rpm_motor2 = ((delta_ticks / odom_counts_per_rev) / dtm);

    odom_prev_update_time = current_time;
    odom_prev_left_pulse = odom_left_pulse;
    odom_prev_right_pulse = odom_right_pulse;
}

void PIDCallback(const lino_msgs::PID &pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    char buffer[50];
    sprintf(buffer, "pid.p: %f pid.i: %f pid.d: %f", pid.p, pid.i, pid.d);
    nh.loginfo(buffer);

    // motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    // motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    // motor1pid.tune(pid.p, pid.i, pid.d);
    // motor2pid.tune(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist &cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    // dtostrf(g_req_linear_vel_x, 4, 2, buffer_temp);
    // sprintf(buffer, "g_req_linear_vel_x: %s", buffer_temp);
    // nh.loginfo(buffer);
    // dtostrf(g_req_linear_vel_y, 4, 2, buffer_temp);
    // sprintf(buffer, "g_req_linear_vel_y: %s", buffer_temp);
    // nh.loginfo(buffer);
    // dtostrf(g_req_angular_vel_z, 4, 2, buffer_temp);
    // sprintf(buffer, "g_req_angular_vel_z: %s", buffer_temp);
    // nh.loginfo(buffer);

    g_prev_command_time = millis();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void moveBase()
{

    //get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);


    int current_rpm1 = 0;
    int current_rpm2 = 0;
    getRPM(current_rpm1, current_rpm2);    
    
    //char buffer[50];
    //sprintf(buffer, "req_rpm.motor1: %d current_rpm1: %d compute_rpm1: %d", req_rpm.motor1, current_rpm1, compute_rpm1);
    //nh.loginfo(buffer);
    //sprintf(buffer, "req_rpm.motor2: %d current_rpm2: %d compute_rpm2: %d", req_rpm.motor2, current_rpm2, compute_rpm2);
    //nh.loginfo(buffer);


    // int rpm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
    // int rpm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
    // motor1_controller.spin(req_rpm.motor1);
    // motor2_controller.spin(req_rpm.motor2);
    
    // Kinematics::velocities current_vel;


    // current_vel = kinematics.getVelocities(g_req_angular_vel_z, current_rpm1, current_rpm2);
    //int compute_rpm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
    //int compute_rpm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);

    // motor1pid.setpoint(req_rpm.motor1); 
    // motor2pid.setpoint(req_rpm.motor2); 
    // int compute_rpm1 = motor1pid.compute(current_rpm1);
    // int compute_rpm2 = motor1pid.compute(current_rpm2);

    //Con PID activo no funciona correctamente
    int compute_rpm1 = req_rpm.motor1;
    int compute_rpm2 = req_rpm.motor2;
    
    
    // char buffer[50];
    // sprintf(buffer, "req_rpm.motor1: %d current_rpm1: %d compute_rpm1: %d", req_rpm.motor1, current_rpm1, compute_rpm1);
    // nh.loginfo(buffer);
    // sprintf(buffer, "req_rpm.motor2: %d current_rpm2: %d compute_rpm2: %d", req_rpm.motor2, current_rpm2, compute_rpm2);
    // nh.loginfo(buffer);


    // motor1_controller.spin(compute_rpm1);
    // motor2_controller.spin(compute_rpm2);

    motor1_controller.spin(compute_rpm1);
    motor2_controller.spin(compute_rpm2);

    // Kinematics::velocities current_vel;
    // if (kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
    // {
    //     float current_steering_angle;

    //     current_steering_angle = steer(g_req_angular_vel_z);
    //     current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
    // }
    // else
    // {
    //     current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, 0, 0);
    // }

    Kinematics::velocities current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, 0, 0);

    //pass velocities to publisher object
    raw_vel_msg.linear_x = current_vel.linear_x;
    raw_vel_msg.linear_y = current_vel.linear_y;
    raw_vel_msg.angular_z = current_vel.angular_z;

    //publish raw_vel_msg
    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishIMU()
{
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
}

void printDebug()
{
    // char buffer[50];

    // sprintf(buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
    // nh.loginfo(buffer);
    // sprintf(buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
    // nh.loginfo(buffer);
    // sprintf(buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
    // nh.loginfo(buffer);
    // sprintf(buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
    // nh.loginfo(buffer);
}

void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("BASE CONNECTED");
    delay(1);

    pinMode(ENCODER_LEFT_A, INPUT);
    pinMode(ENCODER_RIGHT_A, INPUT);

    attachPCINT(digitalPinToPCINT(ENCODER_LEFT_B), EncoderLeftCallBack, CHANGE);
    attachPCINT(digitalPinToPCINT(ENCODER_RIGHT_B), EncoderRightCallBack, CHANGE);

    //PWM_MIN, PWM_MAX, K_P, K_I, K_D
    // motor1pid.begin();          
    // motor1pid.tune(K_P, K_I, K_D);
    // motor1pid.limit(PWM_MIN, PWM_MAX);
    // motor2pid.begin();          
    // motor2pid.tune(K_P, K_I, K_D);
    // motor2pid.limit(PWM_MIN, PWM_MAX);

    nh.loginfo("SETUP FIN");
}

void loop()
{
    static unsigned long prev_control_time = 0;
    static unsigned long prev_imu_time = 0;
    //static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if (imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            publishIMU();
        }
        prev_imu_time = millis();
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}




