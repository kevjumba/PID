// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/*
I2Cdev device library code is placed under the MIT license
*/

//imports
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

//global variables
int leftServoPin = 1;  //port number for left motor
int rightServoPin = 2; //port numbersfor right motor
int pulse = 1500; //delay in microseconds

int right_motor_speed_pin = 3;
int right_motor_forward_pin = 4;
int right_motor_backward_pin = 5;
 
int left_motor_speed_pin = 8;
int left_motor_forward_pin = 9;
int left_motor_backward_pin = 10;


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define LED_PIN 13
bool blinkState = false;

//i2c slave address AD0 pin 9 at b1101000 and b1101001
void setup(){
  Wire.begin();
  pinMode(leftServoPin, OUTPUT);
  pinMode(rightServoPin, OUTPUT);
  
  pinMode(right_motor_speed_pin, OUTPUT);
  pinMode(right_motor_forward_pin, OUTPUT);
  pinMode(right_motor_backward_pin, OUTPUT);
  pinMode(left_motor_speed_pin, OUTPUT);
  pinMode(left_motor_forward_pin, OUTPUT);
  pinMode(left_motor_backward_pin, OUTPUT);
  
  //set all motors to off by default
  off(left_motor_speed_pin);
  off(left_motor_forward_pin);
  off(left_motor_backward_pin);
  off(right_motor_speed_pin);
  off(right_motor_forward_pin);
  off(right_motor_backward_pin);

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}


void loop() {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  double desiredTilt = 0;
  double currentTilt = getCurrentTilt();
  double tiltRate = getCurrentTiltRate();
  //constants are random guesses from robot tuning this year
  double kP = 1;
  double kI = 0;
  double kD = 0.5; 
  double torq = pid(currentTilt-desiredTilt, 0, currentTilt + tiltRate, kP, kI, kD);

  //go(2500,2500, 2000);
 // delay(2000);
}

double getCurrentTilt(){
  //get current tilt from gyro
}

double getCurrentTiltRate(){
  //get the rate possibly through gyro, or we can manually calculate it
}

double pid(double error, double integral, double derivative, double Kp, double Ki, double Kd) {
  return -(error * Kp + integral * Ki + derivative * Kd);
}

void on(int pin){
  digitalWrite(pin, HIGH);
}
 
void off(int pin){
  digitalWrite(pin, LOW);
}

void go(int left_motor_speed, int right_motor_speed, int time){
  set_motor(left_motor_speed_pin, left_motor_forward_pin, left_motor_backward_pin, left_motor_speed);
  set_motor(right_motor_speed_pin, right_motor_forward_pin, right_motor_backward_pin, right_motor_speed);
  delay(time);
  set_motor(left_motor_speed_pin, left_motor_forward_pin, left_motor_backward_pin, 0);
  set_motor(right_motor_speed_pin, right_motor_forward_pin, right_motor_backward_pin, 0);
}
 

void set_motor(int speed_pin, int forward_pin, int backward_pin, int speed){
  if(speed > 0){
    off(backward_pin);
    on(forward_pin);
  }else if(speed < 0){
    off(forward_pin);
    on(backward_pin);
    speed = -speed;
  }else{ // speed is 0
    off(forward_pin);
    off(backward_pin);
  }if(speed > 255){
    speed = 255;
  }
  analogWrite(speed_pin, speed);
}
 
 
