#define USE_PCA9685_SERVO_EXPANDER
#define MAX_EASING_SERVOS 5
#define ENABLE_MICROS_AS_DEGREE_PARAMETER
#define THRESHOLD_VALUE_FOR_INTERPRETING_VALUE_AS_MICROSECONDS  185
#include "ServoEasing.hpp"

#define DEBUG false
// base motor
#define ENCODER_PIN 2
#define STOP_MOTOR  290 //260
#define CW_FAST 80
#define CW_SLOW 250
#define CCW_FAST 450
#define CCW_SLOW 280

// joint servos
ServoEasing * servo0 = new ServoEasing(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing * servo1 = new ServoEasing(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing * servo2 = new ServoEasing(PCA9685_DEFAULT_ADDRESS, &Wire);
ServoEasing * servo3 = new ServoEasing(PCA9685_DEFAULT_ADDRESS, &Wire);

// Joint lengths (cm) - Angles (degrees)
double len[4] = {0, 8, 8, 4}; // 0 is origin
double angles[4] = {0, 0, 0, 0}; //angle from joint 0 to 3
double x[3] = {0, 0, 0}; // 0 is origin
double y[3] = {0, 0, 0};
double x_f, y_f, angle_f; //end effector

// base joint
const byte encoderPin = 2; //int pin
int motor_degree = 0;
int motor_speed = 0;
float diskslots = 20;
float degree_slot = 360/diskslots;
 
void motor_count()  
{  
  if (motor_speed < 0) { //CW
    motor_degree+=degree_slot;
  }
  if(motor_speed > 0){ //CCW
    motor_degree-=degree_slot;
  }
  Serial.print("Motor turn: "); 
  Serial.println(motor_degree);
} 

void setDegree(int degree) {
  if(degree > 0) //round to nearest degree slot
    degree = (int)((degree+(degree_slot/2))/degree_slot) * degree_slot;
  if(degree < 0)
    degree = (int)((degree-(degree_slot/2))/degree_slot) * degree_slot;
  int motor_input = map(motor_degree-degree, -180, 180, 200, 380);
  if(motor_degree == degree){
    motor_speed = 0;
  }
  else if(motor_degree < degree) { //turn cw
    motor_speed = -1;
  } else { //turn ccw
    motor_speed = 1;
  }
}

int compServoAngle(int angle) { //compensate  for servo error
  if((angle>=20 && angle<=50) || (angle>=130 && angle<=160))
    return angle-5;
  else if((angle>=50 && angle<=70) || (angle>=110 && angle<130))
    return angle-7;
  else if(angle>70 && angle<110)
    return angle-10;
  else
    return angle;
}

double toDeg(double angle) {
  return angle*180/M_PI;
}

double toRad(double angle) {
  return angle*M_PI/180;
}

void calculateIK(double x_f,double y_f, double angle_f) {
  x[2] = x_f + len[3]*cos(toRad(angle_f));
  y[2] = y_f + len[3]*sin(toRad(angle_f));
  double c = sqrt(x[2]*x[2]+y[2]*y[2]); // dist from origin to (x2,y2)
  angles[1]=atan(y[2]/x[2]) + acos((len[2]*len[2]-len[1]*len[1]-c*c)/(-2*len[1]*c));
  
  x[1] = len[1]*cos(angles[1]);
  y[1] = len[1]*sin(angles[1]);
  
  angles[2] = acos((c*c-len[1]*len[1]-len[2]*len[2])/(-2*len[1]*len[2]));

  double d = sqrt((x_f-x[1])*(x_f-x[1])+(y_f-y[1])*(y_f-y[1]));
  
  angles[3] = acos((d*d-len[2]*len[2]-len[3]*len[3])/(-2*len[2]*len[3]));

  // flip angles for motors CCW convention
  angles[1] = M_PI-angles[1];
  angles[2] = M_PI-angles[2];
  angles[3] = M_PI-angles[3];
  // convert to degrees
  angles[1] = toDeg(angles[1]);
  angles[2] = toDeg(angles[2]);
  angles[3] = toDeg(angles[3]);
  #if DEBUG
    Serial.print(angles[1]);
    Serial.print(" ");
    Serial.print(angles[2]);
    Serial.print(" ");
    Serial.println(angles[3]);
  #endif
}

void moveMotors(double x_f,double y_f, double angle_f, int angle_speed) {
  setSpeedForAllServos(angle_speed);
  calculateIK(x_f, y_f, angle_f);
  if(!isnan(angles[1]) && !isnan(angles[2]) && !isnan(angles[3])) {
    for(int i = 1; i <= ServoEasing::sServoArrayMaxIndex; ++i) {
      ServoEasing::ServoEasingArray[i]->startEaseTo(angles[i], angle_speed);
    }
  }
}

void returnHome() {
  ServoEasing::ServoEasingArray[0]->write(STOP_MOTOR);
  ServoEasing::ServoEasingArray[1]->write(10);
  ServoEasing::ServoEasingArray[2]->write(170);
  ServoEasing::ServoEasingArray[3]->write(0);
}
void setup() {
  #if DEBUG
    Serial.begin(9600);
  #endif

  Wire.begin();
  // Servo CCW convention
  servo0->attach(0); // set using microseconds
  servo1->attach(1, 2000, 500, 0, 180);
  servo2->attach(2, 500, 2000, 0, 180);
  servo3->attach(3, 2000, 500, 0, 180);
  // initial resting position
  returnHome();
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), motor_count, RISING);
  delay(2000);
}
unsigned long timer = 0;
double add = 0;
bool done[2] = {false, false};
void loop() {
  // Move the motors to desired x and y location in cm
  moveMotors(5, 6, 180, 350);
  delay(1000);
  moveMotors(10, 6, 180, 350);
  delay(1000);
  moveMotors(10, 10, 180, 350);
  delay(1000);
  moveMotors(5, 10, 180, 350);
  delay(1000);
}
