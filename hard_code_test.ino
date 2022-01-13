//Include required libraries
#include <PPMReader.h>
#include <Servo.h>

//ps:
//6cm: deviation to run 110cm in 255 speed from the center line, 110 is for the mid of car to reach destination line, car head alinn with starting zone edge

//Instantiate servo motor objects, names can be changed
//Servo servo_Name;
Servo servo_base1;
Servo servo_base2;
Servo servo_gripper;

float pos_base1 = 0;
float pos_base2 = 0;
float pos_gripper = 0;

//default: 127-255
int l_max_speed = 255;
int l_min_speed = 127;

int r_max_speed = 255;
int r_min_speed = 127;

//Global Pins Assignment for motor pins, and motor enable pins
//IN1-IN4 pins can be connected to both digital and analog pins
//EN1-EN2 pins to be connected to D3, D5, D6, D9, D10, or D11
//const int pin_Name = pin_Number;
const int FL_FwdPin = A4;
const int FL_BwdPin = A5;
const int BL_FwdPin = 13;
const int BL_BwdPin = 12;
const int L_EN = 5;
const int FR_FwdPin = A3;
const int FR_BwdPin = A2;
const int BR_FwdPin = A1;
const int BR_BwdPin = A0;
const int R_EN = 6;

//Controller settings, do not change
byte interruptPin = 3;
byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);

//Initialize global variables
//These are used in the calculation for the robot's wheel speed
//when it is turning left/right
int turn_Speed = 0;
int L_Speed = 0;
int R_Speed = 0;
bool Dir = true; //Forward = true, Backward = false

//angle for picking balls
int base_1_low = 167;
int base_2_low = 13;
int base_1_high = 87;
int base_2_high = 93;

// grab balls
int ball_prepare = 55;
int ball_pick = 45;

// grab cubes
int cube_prepare = 90;
int cube_pick = 71;

//ultra
float dist;
float d;

//speed when auto
float v_f = 46.2;// 1cm/sec,go forward 4.33 for 200cm
float v_b = 48.0;// 1cm/sec,go backward 4.1666 for 2m
float w_l = 136.7;// 1 degree/sec, turn left 13.166 for 5 rounds
float w_r = 130.1;// 1 degree/sec, turn right 13.833 for 5 rounds

//deviation angle when going forward (100cm for 6cm)
float angle_dev = 90 - atan(100/6);

//ultrasonic sensor 
int echoPin = 7;
int trigPin = 8;
long duration; // variable for the duration of sound wave travel
float distance; // variable for the distance measurement

//auto time calc
float now;
float now_init;
float displacement;

void setup() {

  //Setting the L298N pins as output pins
  //pinMode(pin_Name, OUTPUT);
  pinMode(FL_FwdPin, OUTPUT);
  pinMode(FL_BwdPin, OUTPUT);
  pinMode(BL_FwdPin, OUTPUT);
  pinMode(BL_BwdPin, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(FR_FwdPin, OUTPUT);
  pinMode(FR_BwdPin, OUTPUT);
  pinMode(BR_FwdPin, OUTPUT);
  pinMode(BR_BwdPin, OUTPUT);
  pinMode(R_EN, OUTPUT);
  
  //ultra
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  //Start serial comm for troubleshooting through serial monitor
  //Serial.begin(baud rate);
  Serial.begin(115200);
}

void loop() {
  int swb = readChannel(7);
  if (swb >= 1400 && swb < 1600) // swb is in the middle
  {
  start_to_pick_blue_ball_blueside();
}
else{
  leftStop();
  rightStop();
}
}

//auto

//go forward for straight line distance in cm, speed in sec
void go_forward_def_dis(float displacement){
  float duration = displacement/v_f;
  now_init = millis();
  now = now_init;
  while ((now-now_init)<1000*duration){
    go_forward();
    now = millis();}
  delay(200);
}

//go backward for straight line distance in cm, speed in sec
void go_backward_def_dis(float displacement){
  float duration = displacement/v_b;
  for (int times = 0; times < duration; times++){
    //1 second
    for (int count = 0; count < 200000; count++){go_backward();}
  }
  delay(200);
}

//turn right for some angle in degree, speed in sec
void turn_right_def_angle(float angle){
  float duration = angle/w_r;
  for (int times = 0; times < duration; times++){
    //1 second
    for (int count = 0; count < 200000; count++){turn_right();}
  }
  delay(200);
}

//turn left for some angle in degree, speed in sec
void turn_left_def_angle(float angle){
  float duration = angle/w_l;
  for (int times = 0; times < duration; times++){
    //1 second
    for (int count = 0; count < 200000; count++){turn_left();}
  }
  delay(200);
}

void pick(){  
//lower down the arm
  pos_base1 = base_1_low;
  pos_base2 = base_2_low;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
  
  //prepare to pick
  servo_gripper.write(ball_prepare);
  
  //pick
  servo_gripper.write(ball_pick);
  
  //lift arm to shoulder_picked
  pos_base1 = base_1_high;
  pos_base2 = base_2_high;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
}

void release_ball(){
  servo_gripper.write(ball_prepare);
  }
  
float ultra_read(){
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void start_to_pick_blue_ball_blueside(){
  //1m per 6cm deviation

  /*step1*/
  //110cm, from start to reaching blue ball center line
  go_forward_def_dis(110);

  /*step2*/
  //turn right
  turn_right_def_angle(90-angle_dev);

  /*step3*/
  //calc dist and pick a blue ball
  dist = 7;//dist btw wall and ultra
  d = 0; 
  while ((d>=dist) and (d>=0.1)){
    go_forward();
    d = ultra_read();}  
  pick();

  /*step4*/
  //turn right
  turn_right_def_angle(90);
  
  /*step5*/
  //go straight back to starting zone, deviate 
  go_forward_def_dis(110);

  /*step6*/
  //turn right
  turn_right_def_angle(90-angle_dev);

  /*step7*/
  //go straight back to starting zone, deviate 
  go_forward_def_dis(110);

  /*step7*/
  //turn left
  turn_right_def_angle(90-angle_dev);

  /*step8*/
  //calc dist and pick a blue ball
  dist = 7;//dist btw stovetop and ultra
  d = 0; 
  while ((d>=dist) and (d>=0.1)){
    go_forward();
    d = ultra_read();}
    
  /*step9*/
  //put ball on stovetop
  release_ball();

  /*step10*/
  //go back to green ball center line
  
  }

void speed_Check(bool dir, int throttle_Val,int steering_Val, int l_Limit, int r_Limit) {
  //Set minimum value for the motors to be 127
  //Map the input values from the controller to the
  //motors' speed value
  L_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
  R_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);

  //If the right stick is pushed left,
  //1. robot will steer left if moving forward
  //2. robot will steer right if moving backwards
  if (steering_Val >= 1000 and steering_Val < 1400){
    turn_Speed = map(steering_Val, 1400, 1000, 0, 127);
    L_Speed = abs(L_Speed + (-2 * dir + 1)*turn_Speed);
    R_Speed = abs(R_Speed + (2 * dir - 1)*turn_Speed);
  }
  //If the right stick is pushed right,
  //1. robot will steer right if moving forward
  //2. robot will steer left if moving backwards
  else if (steering_Val <= 2000 and steering_Val > 1600){
    turn_Speed = map(steering_Val, 1600, 2000, 0, 127);
    L_Speed = abs(L_Speed + (2 * dir - 1)*turn_Speed);
    R_Speed = abs(R_Speed + (-2 * dir + 1)*turn_Speed);
  }
  //if the right stick is stationary, continue as per normal
  else{
    L_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
    R_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
  }

  //Limit the wheel's speed value to 255
  if(L_Speed > 255){L_Speed = 255;}
  if(R_Speed > 255){R_Speed = 255;}
  //Control the wheel's speed via PWM
  analogWrite(L_EN, L_Speed);
  analogWrite(R_EN, R_Speed);
}

void rotate(int rotate_Val, int l_Limit, int r_Limit){
    L_Speed = map(rotate_Val, l_Limit, r_Limit, 127, 255);
    R_Speed = map(rotate_Val, l_Limit, r_Limit, 127, 255);
    analogWrite(L_EN, L_Speed);
    analogWrite(R_EN, R_Speed);
}


void leftFwd() {
  digitalWrite(FL_FwdPin, HIGH);
  digitalWrite(FL_BwdPin, LOW);
  digitalWrite(BL_FwdPin, HIGH);
  digitalWrite(BL_BwdPin, LOW);
}

void leftBwd() {
  digitalWrite(FL_FwdPin, LOW);
  digitalWrite(FL_BwdPin, HIGH);
  digitalWrite(BL_FwdPin, LOW);
  digitalWrite(BL_BwdPin, HIGH);
}

void leftStop() {
  digitalWrite(FL_FwdPin, LOW);
  digitalWrite(FL_BwdPin, LOW);
  digitalWrite(BL_FwdPin, LOW);
  digitalWrite(BL_BwdPin, LOW);
}


void rightFwd() {
  digitalWrite(FR_FwdPin, HIGH);
  digitalWrite(FR_BwdPin, LOW);
  digitalWrite(BR_FwdPin, HIGH);
  digitalWrite(BR_BwdPin, LOW);
}

void rightBwd() {
  digitalWrite(FR_FwdPin, LOW);
  digitalWrite(FR_BwdPin, HIGH);
  digitalWrite(BR_FwdPin, LOW);
  digitalWrite(BR_BwdPin, HIGH);
}

void rightStop() {
  digitalWrite(FR_FwdPin, LOW);
  digitalWrite(FR_BwdPin, LOW);
  digitalWrite(BR_FwdPin, LOW);
  digitalWrite(BR_BwdPin, LOW);
}





void turn_right(){
    //turn right when going forward
    leftFwd();
    rightBwd();
    rotate(1600, 1550, 2000);
}

void turn_left(){
    //turn left when going forward
    rightFwd();
    leftBwd();
    rotate(1400, 1550, 2000);
}

void go_forward(){
  //forward
    leftFwd();
    rightFwd();
    Dir = true;
    speed_Check(Dir, 1600, 1500, 1550, 2000);

}

void go_backward(){
  //backward
    leftBwd();
    rightBwd();
    Dir = false;
    speed_Check(Dir, 1600, 1500, 1550, 2000);
}


int readChannel(int channelNumber) {
  unsigned value = ppm.latestValidChannelValue(channelNumber, 0);
  //Serial.print(value);
  //Serial.println(", ");
  return value;
}

