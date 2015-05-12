int leftServoPin = 1;  //port number for left motor
int rightServoPin = 2; //port numbersfor right motor
int pulse = 1500; //delay in microseconds

int right_motor_speed_pin = 3;
int right_motor_forward_pin = 4;
int right_motor_backward_pin = 5;
 
int left_motor_speed_pin = 8;
int left_motor_forward_pin = 9;
int left_motor_backward_pin = 10;
 
//i2c slave address AD0 pin 9 at b1101000 and b1101001
void setup(){
  pinMode(leftServoPin, OUTPUT);
  pinMode(rightServoPin, OUTPUT);
  
  pinMode(right_motor_speed_pin, OUTPUT);
  pinMode(right_motor_forward_pin, OUTPUT);
  pinMode(right_motor_backward_pin, OUTPUT);
  pinMode(left_motor_speed_pin, OUTPUT);
  pinMode(left_motor_forward_pin, OUTPUT);
  pinMode(left_motor_backward_pin, OUTPUT);}
  
  Serial.begin(9600);
}
  //rishab is stupid
void loop() {
  double desiredTilt = 0;
  double currentTilt = getCurrentTilt();
  double tiltRate = getCurrentTiltRate();
  //constants are random guesses from robot tuning this year
  double kP = 1;
  double kI = 0;
  double kD = 0.5; 
  double torq = pid(currentTilt-desiredTilt, 0, currentTilt + currentTiltRate, kP, kI, kD);
}

double getCurrentTilt(){
  //get current tilt from gyro
}

double getCurrentTiltRate(){
  //get the rate possibly through gyro, or we can manually calculate it
}
double pid(error, integral, derivative, Kp, Ki, Kd) {
  return -(error * Kp + integral * Ki + derivative * Kd);
}

void on(int pin){
  digitalWrite(pin, HIGH);
}
 
void off(int pin){
  digitalWrite(pin, LOW);
}
 
