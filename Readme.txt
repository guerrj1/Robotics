CPE 476 Mobile Robotics

Team Members: Jett Guerrero, David Flores, Ivan Soto


1. Developing a robotic rover using Robotics Operating System  

Mobile robot uses the Romi chassis kit, Romi 32U4 control board, Romi encoder pair kit, builtin Romi motor drivers and Raspberry Pi 3. Robot will be operated using ROS on a virtual machine. 
Data reference: pololu.com.product/3544/resources 
Instruction/program reference: github.com/venki666/CpE476_demos
Pictures and other specifications: CPE 476 Classwork & Final Robot.pdf 



2. Perform basic tests for the motor for going forward and backwards

Rover is programmed to go forward then backwards then turn left/right 

Video link: https://youtu.be/Ipu5MZvm98o 

Code:
#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

void setup()
{
  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);
}

void loop()
{
  // Run both motors forward.
  ledYellow(1);
  for (int speed = 0; speed <= 400; speed++)
  {
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(speed);
    delay(2);
  }
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(speed);
    delay(2);
  }

  // Run both motors backward.
  ledYellow(0);
  for (int speed = 0; speed >= -400; speed--)
  {
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(speed);
    delay(2);
  }
  for (int speed = -400; speed <= 0; speed++)
  {
    motors.setLeftSpeed(speed);
    motors.setRightSpeed(speed);
    delay(2);
  }

  // Run left motor forward.
  ledYellow(1);
  for (int speed = 0; speed <= 400; speed++)
  {
    motors.setLeftSpeed(speed);
    delay(2);
  }
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setLeftSpeed(speed);
    delay(2);
  }

  // Run right motor forward.
  ledYellow(1);
  for (int speed = 0; speed <= 400; speed++)
  {
    motors.setRightSpeed(speed);
    delay(2);
  }
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setRightSpeed(speed);
    delay(2);
  }

  
  delay(500);
}



3. Test and modify encoder code to allow the robot to go in a straight line 

Rover is programmed with an encoder to keep both motors the same to allow the rover to go in a straight line. Each motor corrects itself in relation to the other. 

Video link: https://youtu.be/C_U42Z4iRHY

Code:
#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;

void setup()
{
  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);
}

void loop()
{
  int16_t countsLeft = encoders.getCountsLeft();
  int16_t countsRight = encoders.getCountsRight();


  motors.setSpeeds(100, 100);
    if(countsLeft < countsRight)
    {
      motors.setSpeeds(100*4, 100);
    }

    else if(countsLeft > countsRight)
    {
      motors.setSpeeds(100, 100*4);
    }

}



4. Program robot to perform a square path using four straight line segments with four rotations points.

Rover is programmed to go straight then stop then to make a left turn and continue the same process forming a square path. 

Video link: https://youtu.be/Ax98PttBB6E

Code: 
#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;

void setup()
{
  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);
}

void loop()
{
  // Run left and right motor forward.
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  //go
  motors.setSpeeds(200,201);
  delay(1300);
  motors.setSpeeds(0,0);
  delay(1000);
  //turn
  motors.setSpeeds(0,100);
  delay(1100);
  motors.setSpeeds(0,0);
  delay(1000);
}



5. PID Control

Implementation of PID control to offset unkown tracking errors using the supplied K values was performed using the Arduino code. 

Code: 
#include <Romi32U4.h>

Romi32U4Motors motors;
Romi32U4ButtonA buttonA;
Romi32U4Encoders encoders;

//============================================ IO Pins ============================================
#define rEncoder 2
#define lEncoder 3

#define lpwm 6
#define ldir1 4
#define ldir2 7

#define rpwm 5
#define rdir1 8
#define rdir2 9

//======================================== Global Variables =======================================
// Variables used in interrupts
volatile unsigned long rightWheel;

volatile unsigned long leftWheel;

// Left and Right Initial start speeds
byte rightSpeed = 225;
byte leftSpeed = 225;

// PID Values
double input, output, leftLastError, poportional, derivative, rightLastError;
double rightIntegral = 0;
double leftIntegral = 0;

// pidcount is used to divide the total error (integral formula)
int pidcount = 1;

// PID Multipliers
double kp = 2;
double ki = 0.3;
double kd = 0.5;

// The setpoint is used in the PID equation
double setPoint = 30;

//============================================= Setup =============================================
void setup(){
 
  Serial.begin(9600);
 
  pinMode(rEncoder, INPUT);
  pinMode(lEncoder, INPUT);
 
  pinMode(rpwm, OUTPUT);
  pinMode(rdir1, OUTPUT);
  pinMode(rdir2, OUTPUT);
 
  pinMode(lpwm, OUTPUT);
  pinMode(ldir1, OUTPUT);
  pinMode(ldir2, OUTPUT);
 
  // Enable the pull up resistors for the encoders
  digitalWrite(rEncoder, HIGH);
  digitalWrite(lEncoder, HIGH);
  // Enable interrupts 0 and 1 on pins D2 and D3
  attachInterrupt(0, rightEncoderISR, FALLING);
  attachInterrupt(1, leftEncoderISR, FALLING);
 
  digitalWrite(ldir1, LOW);
  digitalWrite(ldir2, HIGH);
 
  digitalWrite(rdir1, LOW);
  digitalWrite(rdir2, HIGH);

}// End Setup

//============================================= Loop ==============================================
void loop(){

  //========== Right ==========
  // Move encoder count to input and reset to 0
  input = rightWheel;
  rightWheel = 0;
 
  // Calculate the PID values
  poportional = setPoint - input;
  derivative = poportional - rightLastError;
  rightIntegral = (rightIntegral + poportional)/pidcount;
 
  // Scale the PID values and save total as output
  output = kp * poportional + kd * derivative + ki * rightIntegral;
 
  // Save variables for next time
  rightLastError = poportional;
 
  // Add fanal value to speed only if value is lower then 255
  if((rightSpeed + output) > 255) rightSpeed = 255;
  else rightSpeed = output + rightSpeed;
 
  // Finally, set the updated value as new speed
  analogWrite(rpwm, rightSpeed);
 
  //========== Left ==========
  // Move encoder count to input and reset to 0
  input = leftWheel;
  leftWheel = 0;
 
  // Calculate the PID values
  poportional = setPoint - input;
  derivative = poportional - leftLastError;
  leftIntegral = (leftIntegral + poportional)/pidcount;
 
  // Scale the PID values and save total as output
  output = kp * poportional + kd * derivative + ki * leftIntegral;
 
  // Save variables for next time
  leftLastError = poportional;
  pidcount++;
 
  // Add fanal value to speed only if value is lower then 255
  if((leftSpeed + output) > 255) leftSpeed = 255;
  else leftSpeed = output + leftSpeed;
 
  // Finally, set the updated value as new speed
  analogWrite(lpwm, leftSpeed);
 
  delay(100);
 
} 
 


8. Using Rviz

Screenshot of Rviz with Gazebo: https://imgur.com/a/man5ocM



9. Setting up Rasperry Pi with ROS and OpenCV

Installing the image from Ubiquity robotics already came with ROS and OpenCV preinstalled so further ROS installtion was not required. The flashed microsd only needed to be inserted to the Raspberry Pi. 



10. Control rover using teleop commands on ROS

The code was programmed using the Arduino IDE straight into the Romi32U4. RX and TX from the Romi 32U4 was then connected using an FTDI to a mini usb into the Raspberry Pi.
On the virtual machine terminal, an ssh command was performed to connect to the Raspberry Pi. Once in control of the Raspberry Pi, two terminals were opened to perform two commands.
One on terminal, ROS command: rosrun rosserial_python serial_node.py
On the other terminal, ROS command: rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
On the previous terminal, using u i o j k l m , . on the keyboard will move the robot
q/z increases or decreases max speeds by 10%
w/x increases or decreases only linear speed by 10%
e/c increases or decreases only angular speed by 10%

Video link: https://youtu.be/aYKLrKXD32Q

Code: 
#include <Romi32U4.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>


#define EN_L 9
#define IN1_L 10
#define IN2_L 11

#define EN_R 8
#define IN1_R 12
#define IN2_R 13


double w_r=0, w_l=0;

//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;


ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

void setup(){
 Motors_init();
 nh.initNode();
 nh.subscribe(sub);
}


void loop(){
 MotorL(w_l*10);
 MotorR(w_r*10);
 nh.spinOnce();
}


void Motors_init(){

 pinMode(EN_L, OUTPUT);
 pinMode(EN_R, OUTPUT);
 pinMode(IN1_L, OUTPUT);
 pinMode(IN2_L, OUTPUT);
 pinMode(IN1_R, OUTPUT);
 pinMode(IN2_R, OUTPUT);
 digitalWrite(EN_L, LOW);
 digitalWrite(EN_R, LOW);
 digitalWrite(IN1_L, LOW);
 digitalWrite(IN2_L, LOW);
 digitalWrite(IN1_R, LOW);
 digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){
 if (Pulse_Width1 > 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, HIGH);
     digitalWrite(IN2_L, LOW);
 }

 if (Pulse_Width1 < 0){
     Pulse_Width1=abs(Pulse_Width1);
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, HIGH);
 }

 if (Pulse_Width1 == 0){
     analogWrite(EN_L, Pulse_Width1);
     digitalWrite(IN1_L, LOW);
     digitalWrite(IN2_L, LOW);
 }
}

void MotorR(int Pulse_Width2){
 if (Pulse_Width2 > 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, HIGH);
 }

 if (Pulse_Width2 < 0){
     Pulse_Width2=abs(Pulse_Width2);
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, HIGH);
     digitalWrite(IN2_R, LOW);
 }

 if (Pulse_Width2 == 0){
     analogWrite(EN_R, Pulse_Width2);
     digitalWrite(IN1_R, LOW);
     digitalWrite(IN2_R, LOW);
 }

}


 

