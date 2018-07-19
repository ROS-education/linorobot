#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif

#include <Servo.h>

#define USE_STM32
#define USE_ROS

#include "lino_base_config.h"

#ifdef USE_ROS
#include "ros.h"
#include "ros/time.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Vector3.h"

//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for pid server
#include "lino_msgs/PID.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "Imu.h"
#endif

#include "Motor.h"

#include "Kinematics.h"
#include "PID.h"

/***** Class decraration *****/
#ifdef USE_STM32
  #define EN_PIN_1 12
  #define EN_PIN_2 22

class Encoder {
  public:
    Encoder(int8_t PinA, int8_t PinB);
    void update ();
    long int read () { return position; };
    void write ( const long int p) { position = p; };
    void doEncoderA();
    void doEncoderB();
  private:
    volatile long position;
    int8_t pin_a;
    int8_t pin_b;
    boolean A_set;
    boolean B_set;
};

#else
  #define ENCODER_OPTIMIZE_INTERRUPTS
  #include "Encoder.h"
#endif

/*
class Motor
{
    public:
        enum driver {L298, BTS7960, ESC};
        Motor(driver motor_driver, int counts_per_rev, int pwm_pin, int motor_pinA, int motor_pinB);
        //Motor(int motor_pinA, int motor_pinB);
        void updateSpeed(long encoder_ticks);
        void spin(int pwm);
        int getRPM();
        
        int getRPM2();

    private:
        Servo motor_;
        int rpm_;
        int counts_per_rev_;
        driver motor_driver_;
        long prev_encoder_ticks_;
        unsigned long prev_update_time_;
        int pwm_pin_;
        int motor_pinA_;
        int motor_pinB_;
        
        int smooth_rpm_;
};
*/

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 25 //hz
#define DEBUG_RATE 5

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B); 
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B); 
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B); 

Servo steering_servo;

Motor motor1(Motor::MOTOR_DRIVER, COUNTS_PER_REV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2(Motor::MOTOR_DRIVER, COUNTS_PER_REV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 
Motor motor3(Motor::MOTOR_DRIVER, COUNTS_PER_REV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4(Motor::MOTOR_DRIVER, COUNTS_PER_REV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);//, 1./(float)COMMAND_RATE);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);//, 1./(float)COMMAND_RATE);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);//, 1./(float)COMMAND_RATE);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);//, 1./(float)COMMAND_RATE);

Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE, PWM_BITS);

#ifdef USE_ROS
float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

lino_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

lino_msgs::Velocities acc;
ros::Publisher raw_acc_pub("raw_acc", &acc);

lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);
#endif

#ifdef USE_STM32
/*
void doEncoder1(){
  motor1_encoder.update();
}

void doEncoder2(){
  motor2_encoder.update();
}

void doEncoder3(){
  motor3_encoder.update();
}

void doEncoder4(){
  motor4_encoder.update();
}
*/

void doEncoder1A(){
  motor1_encoder.doEncoderA();
}

void doEncoder2A(){
  motor2_encoder.doEncoderA();
}

void doEncoder1B(){
  motor1_encoder.doEncoderB();
}

void doEncoder2B(){
  motor2_encoder.doEncoderB();
}
#endif

void setup()
{
#ifdef USE_STM32
    //my encoder
    attachInterrupt(MOTOR1_ENCODER_A, doEncoder1A, CHANGE);
    attachInterrupt(MOTOR2_ENCODER_A, doEncoder2A, CHANGE);
    //attachInterrupt(MOTOR3_ENCODER_A, doEncoder3, CHANGE);
    //attachInterrupt(MOTOR4_ENCODER_A, doEncoder4, CHANGE);

    attachInterrupt(MOTOR1_ENCODER_B, doEncoder1B, CHANGE);
    attachInterrupt(MOTOR2_ENCODER_B, doEncoder2B, CHANGE);

    //enable motor VNH2SP30 module
    pinMode(EN_PIN_1, OUTPUT);
    pinMode(EN_PIN_2, OUTPUT);

    digitalWrite(EN_PIN_1, HIGH);
    digitalWrite(EN_PIN_2, HIGH); 
#endif
    
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(90); 
    
#ifdef USE_ROS    
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    //nh.getHardware()->setBaud(115200);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("LINOBASE CONNECTED");
    delay(1);
#else
  motor1.spin(50);
  motor2.spin(50);
#endif
}

void loop()
{
#ifdef USE_ROS
    static unsigned long prev_control_time = 0;
    static unsigned long publish_vel_time = 0;
    static unsigned long prev_imu_time = 0;
    static unsigned long prev_debug_time = 0;
    static bool imu_is_initialized;

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
    {
        //publishVelocities();
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
    }

    //this block publishes velocity based on defined rate
    if ((millis() - publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
    {
        publishVelocities();
        publish_vel_time = millis();
    }

    //this block publishes the IMU data based on defined rate
    if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        //sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            imu_is_initialized = initIMU();

            if(imu_is_initialized)
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

    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
#else
    motor1.updateSpeed(motor1_encoder.read());
    motor2.updateSpeed(motor2_encoder.read());
    
  Serial.print(motor1.getRPM());
  Serial.print("\t");
  Serial.print(motor2.getRPM());
  Serial.print("\n");

  delay(200);
#endif
}

#ifdef USE_ROS
void PIDCallback(const lino_msgs::PID& pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void moveBase()
{
    Kinematics::rpm req_rpm;

    //get the required rpm for each motor based on required velocities, and base used
    req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

    motor1.updateSpeed(motor1_encoder.read());
    motor2.updateSpeed(motor2_encoder.read());
    motor3.updateSpeed(motor3_encoder.read());
    motor4.updateSpeed(motor4_encoder.read());

    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1.spin(motor1_pid.compute(req_rpm.motor1, motor1.getRPM()));
    motor2.spin(motor2_pid.compute(req_rpm.motor2, motor2.getRPM()));
    motor3.spin(motor3_pid.compute(req_rpm.motor3, motor3.getRPM()));  
    motor4.spin(motor4_pid.compute(req_rpm.motor4, motor4.getRPM()));    

    //steer if Ackermann base
    #if LINO_BASE == ACKERMANN
        steer();
    #endif
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}

void publishVelocities()
{
    Kinematics::velocities vel;

    //motor1.updateSpeed(motor1_encoder.read());
    //motor2.updateSpeed(motor2_encoder.read());
    //motor3.updateSpeed(motor3_encoder.read());
    //motor4.updateSpeed(motor4_encoder.read());
    
    //calculate the robot's speed based on rpm reading from each motor and platform used.
    vel = kinematics.getVelocities(motor1.getRPM(), motor2.getRPM(), motor3.getRPM(), motor4.getRPM());

    //pass velocities to publisher object
    raw_vel_msg.linear_x = vel.linear_x;
    raw_vel_msg.linear_y = vel.linear_y;
    raw_vel_msg.angular_z = vel.angular_z;

    //publish raw_vel_msg object to ROS
    raw_vel_pub.publish(&raw_vel_msg);
}

void publishIMU()
{
  
    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readAccelerometer();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readGyroscope();

    //pass accelerometer data to imu object
    raw_imu_msg.magnetic_field = readMagnetometer();

    //publish raw_imu_msg object to ROS
    raw_imu_pub.publish(&raw_imu_msg);

/*
geometry_msgs::Vector3 a;
a = readAccelerometer();
acc.linear_x = (float)a.x;
acc.linear_y = (float)a.y;
acc.angular_z = (float)a.z;
raw_acc_pub.publish(&acc);
*/
/*
    if(DEBUG){
      char buffer[50];

      sprintf (buffer, "Acc: %f\t%f\t%f", raw_imu_msg.linear_acceleration.x, raw_imu_msg.linear_acceleration.y, raw_imu_msg.linear_acceleration.z);
      nh.loginfo(buffer);
      sprintf (buffer, "Gyr: %f\t%f\t%f", raw_imu_msg.angular_velocity.x, raw_imu_msg.angular_velocity.y, raw_imu_msg.angular_velocity.z);
      nh.loginfo(buffer);
      sprintf (buffer, "Mag: %E\t%E\t%E", raw_imu_msg.magnetic_field.x, raw_imu_msg.magnetic_field.y, raw_imu_msg.magnetic_field.z);
      nh.loginfo(buffer);
    }
*/
}

void steer()
{
    //steering function for ACKERMANN base
    //this converts angular velocity(rad) to steering angle(degree)
    float steering_angle;
    float steering_angle_deg;

    float req_steering_angle = g_req_angular_vel_z;

    //convert steering angle from rad to deg
    steering_angle_deg = req_steering_angle * (180 / PI);

    if(steering_angle_deg > 0)
    {
        //steer left 
        steering_angle = mapFloat(steering_angle_deg, 0, 90, 90, 0);

    }
    else if(steering_angle_deg < 0)
    {
        //steer right
        steering_angle = mapFloat(steering_angle_deg, 0, -90, 90, 180);
    }
    else
    {
        //return steering wheel to middle if there's no command
        steering_angle = 90;
    }
    
    //steer the robot
    steering_servo.write(steering_angle);
}

float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void printDebug()
{
    char buffer[50];

    sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
    nh.loginfo(buffer);
    sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
    nh.loginfo(buffer);
}
#endif

/***** Class implementations *****/
//Encoder
#ifdef USE_STM32
Encoder::Encoder( int8_t PinA, int8_t PinB) : pin_a ( PinA), pin_b( PinB ) {
      // set pin a and b to be input
      pinMode(pin_a, INPUT);
      pinMode(pin_b, INPUT);
      // and turn on pull-up resistors
      digitalWrite(pin_a, HIGH);
      digitalWrite(pin_b, HIGH);
    }

void Encoder::update () {
      if (digitalRead(pin_a)) digitalRead(pin_b) ? position++ : position--;
      else digitalRead(pin_b) ? position-- : position++;
    }

    // Interrupt on A changing state
void Encoder::doEncoderA() {
  // Test transition
  A_set = digitalRead(pin_a) == HIGH;
  // and adjust counter + if A leads B
  position += (A_set != B_set) ? +1 : -1;
}

// Interrupt on B changing state
void Encoder::doEncoderB() {
  // Test transition
  B_set = digitalRead(pin_b) == HIGH;
  // and adjust counter + if B follows A
  position += (A_set == B_set) ? +1 : -1;
}
#endif

/*
// Motor
Motor::Motor(driver motor_driver, int counts_per_rev, int pwm_pin, int motor_pinA, int motor_pinB)
{
    motor_driver_ = motor_driver;
    counts_per_rev_ = counts_per_rev;

    pwm_pin_ = pwm_pin;
    motor_pinA_ = motor_pinA;
    motor_pinB_ = motor_pinB;
    
    smooth_rpm_ = 0;
    
    switch (motor_driver)
    {
        case L298:
            pinMode(pwm_pin_, OUTPUT);
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
#ifdef USE_STM32
            pwmWrite(pwm_pin_, abs(0));
#else
            analogWrite(pwm_pin_, abs(0));
#endif

            break;

        case BTS7960:
            pinMode(motor_pinA_, OUTPUT);
            pinMode(motor_pinB_, OUTPUT);

            //ensure that the motor is in neutral state during bootup
#ifdef USE_STM32
            pwmWrite(motor_pinB_, 0);
            pwmWrite(motor_pinA_, 0);
#else
            analogWrite(motor_pinB_, 0);
            analogWrite(motor_pinA_, 0);
#endif

            break;

        case ESC:
            motor_.attach(motor_pinA_);

            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);

            break;
    }
}

void Motor::updateSpeed(long encoder_ticks)
{
    //this function calculates the motor's RPM based on encoder ticks and delta time
    unsigned long current_time = millis();
    unsigned long dt = current_time - prev_update_time_;

    //convert the time from milliseconds to minutes
    double dtm = (double)dt / 60000;
    double delta_ticks = encoder_ticks - prev_encoder_ticks_;

    //calculate wheel's speed (RPM)
    rpm_ = (delta_ticks / counts_per_rev_) / dtm;

    prev_update_time_ = current_time;
    prev_encoder_ticks_ = encoder_ticks;
    
    smooth_rpm_ = (1.-0.95)*rpm_ + 0.95*smooth_rpm_;
}

int Motor::getRPM()
{
    return rpm_;
}

int Motor::getRPM2()
{
    return smooth_rpm_;
}

void Motor::spin(int pwm)
{
    switch (motor_driver_)
    {
        case L298:
            if(pwm > 0)
            {
                digitalWrite(motor_pinA_, HIGH);
                digitalWrite(motor_pinB_, LOW);
            }
            else if(pwm < 0)
            {
                digitalWrite(motor_pinA_, LOW);
                digitalWrite(motor_pinB_, HIGH);
            }
#ifdef USE_STM32
            pwmWrite(pwm_pin_, abs(pwm));
#else
            analogWrite(pwm_pin_, abs(pwm));
#endif

            break;

        case BTS7960:
            if (pwm > 0)
            {
#ifdef USE_STM32
                pwmWrite(motor_pinA_, 0);
                pwmWrite(motor_pinB_, abs(pwm));
#else
                analogWrite(motor_pinA_, 0);
                analogWrite(motor_pinB_, abs(pwm));
#endif
            }
            else if (pwm < 0)
            {
#ifdef USE_STM32
                pwmWrite(motor_pinB_, 0);
                pwmWrite(motor_pinA_, abs(pwm));
#else
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, abs(pwm));
#endif
            }
            else
            {
#ifdef USE_STM32
                pwmWrite(motor_pinB_, 0);
                pwmWrite(motor_pinA_, 0);
#else
                analogWrite(motor_pinB_, 0);
                analogWrite(motor_pinA_, 0);
#endif
            }

            break;
        
        case ESC:
            motor_.writeMicroseconds(1500 + pwm);

            break;
    }
}
*/
