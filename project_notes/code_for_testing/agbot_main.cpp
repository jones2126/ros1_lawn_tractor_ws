#include <WProgram.h>
#include "ros.h"
#include "ros/time.h"
#include <math.h>
#include <TimerOne.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "Servo.h"
#include <SimpleKalmanFilter.h>



Servo transServo;

// cmd_vel setup
float linear_vel_x = 0;
float angular_vel_z = 0;
int debug_vel_y = 0;
float input_vel_z = 0;
int deadband = 175;
float vel_i_gain = 0;


// Steer angle sensor
int steer_angle_val;
int steer_center = 480;
int max_steer_eff = 50;
int steer_left_max = 220;
int steer_right_max = 770;

// Reverse Switch
int reverseSwitch = 5;
int reverseState = 0;

// Encoder Ticks
volatile byte right_ticks;
volatile byte right_interrupt = 0;
int right_encoder_tick_count = 0;
volatile unsigned long time_left, time_right;
double time_left_speed = 0;
double time_right_speed = 0;


volatile byte left_ticks;
volatile byte left_interrupt = 0;
int left_encoder_tick_count = 0;

#define COMMAND_RATE 20 //hz
unsigned long g_prev_command_time = 0;


//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_msg("cmd_vel", commandCallback);


// Publisher of sensors
geometry_msgs::TwistStamped raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);


void left_tick_detect()
{
  left_ticks++;
  time_left = millis();
}

void right_tick_detect()
{
  right_ticks++;
  time_right = millis();
}


void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
    //callback function every time linear and angular speed is received from 'cmd_vel' topic
    //this callback function receives cmd_msg object where linear and angular speed are stored
    linear_vel_x = cmd_msg.linear.x;

    angular_vel_z = cmd_msg.angular.z;

    vel_i_gain = cmd_msg.angular.y;
    
    debug_vel_y = cmd_msg.linear.y;
    input_vel_z = cmd_msg.linear.z;

    g_prev_command_time = millis();
}


float mapFloat(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


double ave_angle_val;
int steer_Ki_count = 0;
int steer_Ki_timeout = 10;
int steer_Ki = 10;
int steer_last_value = 0;
double steer_err_value = 0;

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.08);


void steer()
{

    //this converts angular velocity(rad) to steering angle(degree)
    float steering_angle_target;
    float steering_angle_deg;

    //convert steering angle from rad to deg
    steering_angle_deg = angular_vel_z * (180 / PI);


    // Scale the cmd_vel input to full left and full right of mower
    if(steering_angle_deg > 0)
    {
        //steer left 
        steering_angle_target = mapFloat(steering_angle_deg, 0, 60, steer_center, steer_left_max);
    }
    else if(steering_angle_deg < 0)
    {
        //steer right
        steering_angle_target = mapFloat(steering_angle_deg, 0, -60, steer_center, steer_right_max);
    }
    else
    {
        //return steering wheel to middle if there's no command
        steering_angle_target = steer_center;
    }

    //char buffer[50];
    //sprintf (buffer, "Steer angle value  : %f", steering_angle_target);
    //nh.loginfo(buffer);
     
    steer_angle_val = analogRead(15);

    // steer smoothing
    // read a reference value from A0 and map it from 0 to 100
    float real_value = steer_angle_val/1024.0 * 100.0;
  
    // add a noise to the reference value and use as the measured value
    float measured_value = real_value + random(-100,100)/100.0;

    // calculate the estimated value with Kalman Filter
    float estimated_value = simpleKalmanFilter.updateEstimate(measured_value);

    ave_angle_val = estimated_value * 10.24;

    // Ave or Raw?  Which on is better
    steer_err_value = steering_angle_target - ave_angle_val;
    //steer_err_value = steering_angle_target - steer_angle_val;
    
    int steer_effort = (steer_err_value * steer_Ki) + steer_last_value;
    steer_last_value = steer_effort;

    // Steer I
    steer_Ki_count++;
    if (steer_Ki_count > steer_Ki_timeout){
      steer_Ki_count = 0;
      steer_last_value = 0;
    } 


    // Max steer motor value 255
    if (steer_effort > max_steer_eff){
      steer_effort = max_steer_eff;
    } else if ((steer_effort < 0) && (steer_effort > -max_steer_eff)){
      steer_effort = abs(steer_effort);
    } else if (steer_effort < -max_steer_eff){
      steer_effort = max_steer_eff;
    }

    // set the direction of motor and command motor
    // Only adjust the steer angle if error is greater than value 10
    // Adjust higher or lower depending on how active the steering is (jitter)
    if (steer_err_value < -10){
      digitalWrite(14, HIGH);
      analogWrite(20, abs(steer_effort)); 
    } else if (steer_err_value >10){
      digitalWrite(14, LOW);
      analogWrite(20, abs(steer_effort)); 
    } else {
      analogWrite(20, 0); 
    }
}



// Speed controller settings
// Tried different setups PID

int Ki_count = 0;
int Ki_timeout = 20;
int Ki = 5;
int Kp = 70;
int Kd = 0;
int last_value = 0;
double err_last = 0;
double err_value_Kp = 0;
double error_sum = 0;
double err_value_Kd = 0;
double err_value = 0;
int vel_effort = 0; 
int vel_eff_sum = 0; 
int vel_start = 0;
int vel_eff_target = 0;
int vel_neutral = 1550;

void velocityControl(){


  // Slowly increase the servo until the mower starts moving.
  // That will be the start value 
  if (linear_vel_x > .05) {
    // Start with min value for forward
    vel_start = 225; 
  } else if (linear_vel_x < -.05) {
    // Start with min value for reverse
    vel_start = -200; 
  } else {
    //  Set Transmission to Netural 
    vel_start = 0; 
    vel_eff_sum = 0;
  }

  err_value = linear_vel_x - raw_vel_msg.twist.linear.x;
  err_value_Kp = err_value * Kp;
  err_last = err_last - err_value;
  // err_value_Kd = err_last * Kd;

     
  // Vel I
  
  error_sum = error_sum + err_value;
  vel_effort =  error_sum * Ki;

  vel_eff_target =  vel_effort + vel_start + 1550;
  //vel_eff_target = vel_eff_target + err_value_Kd;
  vel_eff_target = vel_eff_target + err_value_Kp;

  // Add P and D
  //vel_effort = vel_effort + err_value_Kp + err_value_Kd;


  Ki_count++;
  if (Ki_count > Ki_timeout){
    Ki_count = 0;
    error_sum = 0;
  } 

  // Max motor value range check
  if (vel_eff_target > 2100){
    vel_eff_target = 2100;
  } else if ((vel_eff_target < 2100) && (vel_eff_target > 1200)){
    vel_eff_target = abs(vel_eff_target);
  } else if (vel_eff_target < 1200){
    vel_eff_target = 1200;
  }

  // write transmission command
  if (debug_vel_y == 0){
    if (err_value < -0.01){
      transServo.writeMicroseconds(vel_eff_target);
    } else if (err_value > .01){
      transServo.writeMicroseconds(vel_eff_target);
    } else {
      transServo.writeMicroseconds(1550);
    }
  } else if (debug_vel_y == -1){
    transServo.writeMicroseconds(input_vel_z);
  }

  // char buffer[50];
  // sprintf (buffer, "Current Velocity Motor  : %d", vel_effort);
  // nh.loginfo(buffer);

}


void stopBase()
{
  linear_vel_x = 0;
  angular_vel_z = 0;
  //zero_velocity();
}

void moveBase()
{
  //  send motor control
  velocityControl();
  steer();
}


double average_speed = 0;
double left_ave_speed = 0;
double right_ave_speed = 0;
double left_speed = 0;
double right_speed = 0;
double ave_left_ticks = 0;
double ave_right_ticks = 0;


void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  // right_wheel_vel.data = float(counter_right)*2*pi*5/8;
  // right_wheel_vel_pub.publish(&right_wheel_vel);
  // counter_right=0;


  // right tick exp smoothing
  int filterWeight = 16;  //higher number heavier filter
  int numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    ave_right_ticks = ave_right_ticks + (right_ticks - ave_right_ticks) / filterWeight;
  }

  // left tick exp smoothing
  filterWeight = 16;  //higher number heavier filter
  numReadings = 5;
  for (int i = 0; i < numReadings; i++) {
    ave_left_ticks = ave_left_ticks + (left_ticks - ave_left_ticks) / filterWeight;
  }

  double right_speed = ((ave_right_ticks*(2*3.1415*0.2413)/30)/.2);
  double left_speed = ((ave_left_ticks*(2*3.1415*0.2413)/30)/.2);

  // Right Speed exp smoothing
  filterWeight = 20;  //higher number heavier filter
  numReadings = 10;
  for (int i = 0; i < numReadings; i++) {
    right_ave_speed = right_ave_speed + (right_speed - right_ave_speed) / filterWeight;
  }

  // Left Speed exp smoothing
  filterWeight = 20;  //higher number heavier filter
  numReadings = 10;
  for (int i = 0; i < numReadings; i++) {
    left_ave_speed = left_ave_speed + (left_speed - left_ave_speed) / filterWeight;
  }

  // average_speed = (left_ave_speed + right_ave_speed) / 2;
  average_speed = (left_speed + right_speed) / 2;

  reverseState = digitalRead(reverseSwitch); // 1 is forward, 0 is reverse
  if (!reverseState){
    average_speed = average_speed * -1;
    left_ave_speed = left_ave_speed * -1;
    right_ave_speed = right_ave_speed * -1;
    left_speed = left_speed * -1;
    right_speed = right_speed * -1;
  }
  // publish raw speed
  ros::Time header_time = nh.now();
  raw_vel_msg.header.stamp = header_time;
  raw_vel_msg.header.frame_id = "base_link";

  raw_vel_msg.twist.linear.x = average_speed;
  raw_vel_msg.twist.linear.y = left_speed;
  raw_vel_msg.twist.linear.z = right_speed;
  raw_vel_msg.twist.angular.x = steer_angle_val;
  raw_vel_msg.twist.angular.y = error_sum;
  //raw_vel_msg.twist.angular.z = vel_effort;
  raw_vel_msg.twist.angular.z = ave_angle_val;
  raw_vel_pub.publish(&raw_vel_msg);

  left_ticks = 0;
  right_ticks = 0;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}


void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_msg);
  nh.advertise(raw_vel_pub);
  // nh.advertise(teensy_state_pub);
  //nh.serviceClient(client);

  // setup for transmission servo
  transServo.attach(23);
  transServo.writeMicroseconds(1550);

  // Steer angle sensor
  pinMode(15, INPUT);
  // Steer motor
  pinMode(14, OUTPUT);
  pinMode(20, OUTPUT);

  // setup reverse switch
  pinMode(reverseSwitch, INPUT_PULLUP);

  // wheel encoder
  Timer1.initialize(200000);
  attachInterrupt(6, right_tick_detect, FALLING);
  right_ticks = 0;
  attachInterrupt(9, left_tick_detect, FALLING);
  left_ticks = 0;
  Timer1.attachInterrupt( timerIsr ); // enable the timer  

  while (!nh.connected())
  {
      nh.spinOnce();
  }
  nh.loginfo("Tractor is connected");
  delay(1);
}



void loop() {

  if (time_left !=0)
  {
    // speed = distance / time
    time_left_speed = ((2*3.1415*0.2413)/30) / (time_left / 1000.0);
    noInterrupts();
    time_left = 0;
    interrupts();
  }

  if (time_right !=0)
  {
    // speed = distance / time
    time_right_speed = ((2*3.1415*0.2413)/30) / (time_right / 1000.0);
    noInterrupts();
    time_right = 0;
    interrupts();
  }

  static unsigned long prev_control_time = 0;
  //this block drives the robot based on defined rate
  if ((millis() - prev_control_time) >= (1000 / COMMAND_RATE))
  static unsigned long prev_control_time = 0;
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

  //call all the callbacks waiting to be called
  nh.spinOnce();
}



