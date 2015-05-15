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

int integral_time = 4;
double time = 1;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int errorArray [200];

int intIndex = 0;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

int16_t iax, iay, iaz;
int16_t igx, igy, igz;
int16_t imx, imy, imz;

#define LED_PIN 13
bool blinkState = false;
boolean state = false;

double currentIntegral = 0;
double kP = 0.2;
double kI = 0.3;
double kD = 0; 
double compAngleX;
double compAngleY;
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
  
  accelgyro.getMotion9(&iax, &iay, &iaz, &igx, &igy, &igz, &imx, &imy, &imz);
  

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
}
void loop() {
  double start = millis();
  
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  double pitch = (atan2(ay, az)+3.14)*RAD_TO_DEG; //y angle
  double roll = (atan2(ax, az)+3.14)*RAD_TO_DEG; //x angle
  double desiredTilt = 0;
  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * roll);
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * pith);
  currentIntegral = getCurrentTiltIntegral(integral_time);
  double currentTilt = getCurrentTilt();
  //constants are random guesses from robot tuning this year
  //Serial.println(gy);
  kP = 0.2;
  kI = 0.3;
  kD = 0.5; 
  double end = millis();
  time = end - start;
  double futureDerivative = getTiltDerivative(time);
  double torq = pid(currentTilt-desiredTilt, currentIntegral, futureDerivative, kP, kI, kD);
  go(torq, torq, 50);
  // delay(2000);
  printStatus(torq);
  intIndex++;
}

void printStatus(double sol){
  Serial.print(getCurrentTilt());
  Serial.print(" ");
  Serial.print(kP);
  Serial.print("*");
  Serial.print(getCurrentTilt());
  Serial.print(" + ");
  Serial.print(kI);
  Serial.print("*");
  Serial.print(currentIntegral);
  Serial.print(" + ");
  Serial.print(kD);
  Serial.print("*");
  Serial.print(getTiltDerivative(time));
  Serial.print(" = ");
  Serial.println(sol);
}

double getCurrentTiltIntegral(int delta){
  double sum = 0;
  double riemann = 0;
  errorArray[intIndex] = getCurrentTilt();
  int arraySize = 200;
  
  if(intIndex >= arraySize){
   intIndex = delta;
    for(int i = 0; i < delta; i++){
      errorArray[i] = errorArray[arraySize - delta + i];     
    }
  }
  
  if(intIndex < delta){
    for(int i = 0; i < delta; i++){
      sum += errorArray[i];
    }  
    riemann = sum/delta;
  }else{
    sum = 0;
    for(int i = intIndex - delta; i <= intIndex; i++){
      sum = sum + errorArray[i];
    }   
    riemann = sum/delta;
  }
    
   return riemann; 
}

double getCurrentTilt(){
  return compAngleY - igy;
}


double getTiltDerivative(double time) {
  double difference = (errorArray[intIndex] - errorArray[intIndex - 1])/time;
  double difference2 = (errorArray[intIndex-1] - errorArray[intIndex-2])/time;
  double concavity = (difference - difference2)/time;
  double value = difference + (concavity)*time;
  return -value;
  
   
} 
double pid(double error, double integral, double derivative, double Kp, double Ki, double Kd) {
  return (error * Kp + integral * Ki + derivative * Kd);
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
